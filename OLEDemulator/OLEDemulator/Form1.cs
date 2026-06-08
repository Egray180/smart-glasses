// Form1.cs
// - 60x32 OLED emulator (black bg, white ON)
// - Layout:
//   * Pages 0-1 (y=0..15): centered arrow (left/right)
//   * Page 2 (y=16..23): distance + unit (centered)
//   * Page 3 (y=24..31): street (centered, max 8 chars)
// - “Edit Mode” checkbox:
//   * OFF (default): UI inputs render the screen (auto)
//   * ON: clicking toggles pixels for designing icons; auto-render pauses
// - Export: Copy / Save SSD1315/SSD1306 page-major buffer (240 bytes)
// - NEW:
//   * UPDATE SCREEN button sends 14-byte packet over UART
//   * 14 byte display textboxes show what is sent (dec + hex)
//   * Arrow made symmetric in Y

using System;
using System.Collections.Generic;
using System.Drawing;
using System.Drawing.Drawing2D;
using System.IO;
using System.IO.Ports;
using System.Linq;
using System.Text;
using System.Windows.Forms;

namespace OLEDemulator
{
    public partial class Form1 : Form
    {
        // ---------------- Serial ----------------
        ComboBox comselect;
        Button comconnect;
        SerialPort mySerialPort;

        // ---------------- Inputs ----------------
        ComboBox directionCombo;
        TextBox distanceTextbox;
        ComboBox unitCombo;
        TextBox streetTextbox;
        ComboBox specialCombo;

        // ---------------- Packet UI ----------------
        Button updateScreenButton;
        TextBox[] packetByteBoxes = new TextBox[14];

        // ---------------- OLED / Tools ----------------
        Panel oledPanel;
        Button clearOledButton;
        Button copyCArrayButton;
        Button saveCArrayButton;
        CheckBox editModeCheck;

        // OLED geometry
        const int OLED_W = 60;
        const int OLED_H = 32;

        // UI scale: size of each OLED "pixel square"
        const int SCALE = 12;
        const int GRID_LINE_THICKNESS = 1;

        // Pixel buffer: [x,y] true=ON (white)
        readonly bool[,] pixels = new bool[OLED_W, OLED_H];

        // 6x8 font subset (from your asc2_0806; extend anytime)
        static readonly Dictionary<char, byte[]> Font6x8 = BuildFont6x8();

        // ---------------- Packet Encoding Constants ----------------
        // CHANGE THESE IF YOUR FIRMWARE EXPECTS DIFFERENT VALUES.
        const byte START_BYTE = 255;

        const byte DIR_LEFT = 1;
        const byte DIR_RIGHT = 2;

        const byte UNIT_M = 0;
        const byte UNIT_KM = 1;

        const byte SPECIAL_NONE = 0;
        const byte SPECIAL_WRONG_TURN = 1;
        const byte SPECIAL_ACCIDENT = 2;

        public Form1()
        {
            InitializeComponent();
            MyInitialize();
            SetupUI();
        }

        private void MyInitialize()
        {
            Text = "OLED 60x32 Navigation Emulator";
            DoubleBuffered = true;
            StartPosition = FormStartPosition.CenterScreen;
        }

        private void SetupUI()
        {
            SetupSerial();
            SetupControls();

            // Default render
            RenderAll();
            UpdatePacketPreview(); // show bytes immediately
        }

        private void SetupSerial()
        {
            mySerialPort = new SerialPort
            {
                BaudRate = 9600, // placeholder (change if needed)
                DataBits = 8,
                Parity = Parity.None,
                StopBits = StopBits.One,
                Handshake = Handshake.None
            };
        }

        private void SetupControls()
        {
            int pad = 10;

            // ======= Top Serial Bar =======
            comselect = new ComboBox
            {
                Location = new Point(pad, pad),
                Size = new Size(200, 25),
                DropDownStyle = ComboBoxStyle.DropDownList
            };
            comselect.Items.AddRange(SerialPort.GetPortNames());
            if (comselect.Items.Count > 0) comselect.SelectedIndex = 0;
            else comselect.Text = "No COM Ports Found";
            comselect.SelectedIndexChanged += (s, e) =>
            {
                if (comselect.SelectedItem != null)
                    mySerialPort.PortName = comselect.Text;
            };
            Controls.Add(comselect);

            comconnect = new Button
            {
                Text = "Connect",
                Size = new Size(120, 25),
                Location = new Point(comselect.Right + pad, pad)
            };
            comconnect.Click += connectCOM;
            Controls.Add(comconnect);

            // ======= Inputs Group =======
            var inputsGroup = new GroupBox
            {
                Text = "Navigation Inputs",
                Location = new Point(pad, comselect.Bottom + pad),
                Size = new Size(520, 240)
            };
            Controls.Add(inputsGroup);

            int gx = 12;
            int gy = 28;
            int rowH = 34;
            int labelW = 140;
            int controlW = 300;

            inputsGroup.Controls.Add(MkLabel("Direction:", gx, gy, labelW));
            directionCombo = new ComboBox
            {
                Location = new Point(gx + labelW, gy - 3),
                Size = new Size(controlW, 25),
                DropDownStyle = ComboBoxStyle.DropDownList
            };
            directionCombo.Items.AddRange(new object[] { "Left Turn", "Right Turn" });
            directionCombo.SelectedIndex = 0;
            inputsGroup.Controls.Add(directionCombo);

            gy += rowH;
            inputsGroup.Controls.Add(MkLabel("Distance:", gx, gy, labelW));
            distanceTextbox = new TextBox
            {
                Location = new Point(gx + labelW, gy - 3),
                Size = new Size(110, 25)
            };
            // numeric-ish entry (digits + optional dot)
            distanceTextbox.KeyPress += (s, e) =>
            {
                if (!char.IsControl(e.KeyChar) && !char.IsDigit(e.KeyChar) && e.KeyChar != '.')
                    e.Handled = true;
                if (e.KeyChar == '.' && distanceTextbox.Text.Contains('.'))
                    e.Handled = true;
            };
            inputsGroup.Controls.Add(distanceTextbox);

            unitCombo = new ComboBox
            {
                Location = new Point(distanceTextbox.Right + 10, gy - 3),
                Size = new Size(70, 25),
                DropDownStyle = ComboBoxStyle.DropDownList
            };
            unitCombo.Items.AddRange(new object[] { "m", "km" });
            unitCombo.SelectedIndex = 0;
            inputsGroup.Controls.Add(unitCombo);

            gy += rowH;
            inputsGroup.Controls.Add(MkLabel("Street (max 8):", gx, gy, labelW));
            streetTextbox = new TextBox
            {
                Location = new Point(gx + labelW, gy - 3),
                Size = new Size(220, 25),
                MaxLength = 8
            };
            inputsGroup.Controls.Add(streetTextbox);

            gy += rowH;
            inputsGroup.Controls.Add(MkLabel("Special message:", gx, gy, labelW));
            specialCombo = new ComboBox
            {
                Location = new Point(gx + labelW, gy - 3),
                Size = new Size(controlW, 25),
                DropDownStyle = ComboBoxStyle.DropDownList
            };
            specialCombo.Items.AddRange(new object[] { "None", "Wrong turn made", "Accident ahead" });
            specialCombo.SelectedIndex = 0;
            inputsGroup.Controls.Add(specialCombo);

            // UPDATE SCREEN button
            updateScreenButton = new Button
            {
                Text = "UPDATE SCREEN",
                Size = new Size(160, 36),
                Location = new Point(gx + labelW, gy + rowH + 4)
            };
            updateScreenButton.Click += (s, e) => SendNavPacket();
            inputsGroup.Controls.Add(updateScreenButton);

            // ======= Packet Preview Group =======
            var packetGroup = new GroupBox
            {
                Text = "UART Packet Preview (14 bytes)",
                Location = new Point(pad, inputsGroup.Bottom + pad),
                Size = new Size(520, 150)
            };
            Controls.Add(packetGroup);

            // 14 boxes in two rows of 7
            int bx0 = 12;
            int by0 = 30;
            int bw = 65;
            int bh = 25;
            int gap = 6;

            for (int i = 0; i < 14; i++)
            {
                int row = i / 7;
                int col = i % 7;

                var lbl = new Label
                {
                    Text = (i + 1).ToString(),
                    AutoSize = false,
                    TextAlign = ContentAlignment.MiddleCenter,
                    Size = new Size(bw, 16),
                    Location = new Point(bx0 + col * (bw + gap), by0 + row * (bh + 32))
                };
                packetGroup.Controls.Add(lbl);

                var tb = new TextBox
                {
                    ReadOnly = true,
                    TextAlign = HorizontalAlignment.Center,
                    Size = new Size(bw, bh),
                    Location = new Point(bx0 + col * (bw + gap), lbl.Bottom + 2)
                };
                packetByteBoxes[i] = tb;
                packetGroup.Controls.Add(tb);
            }

            var pktHint = new Label
            {
                Text = "Format: dec (0xHH). Distance bytes are 00–99 chunks: B3=(thousands+hundreds), B4=(tens+ones).",
                AutoSize = true,
                Location = new Point(12, packetGroup.Bottom - 26)
            };
            packetGroup.Controls.Add(pktHint);

            // ======= OLED Group =======
            int oledPanelW = OLED_W * SCALE + 1;
            int oledPanelH = OLED_H * SCALE + 1;

            var oledGroup = new GroupBox
            {
                Text = "OLED Preview (60 x 32)",
                Location = new Point(inputsGroup.Right + pad, inputsGroup.Top),
                Size = new Size(oledPanelW + 24, oledPanelH + 120)
            };
            Controls.Add(oledGroup);

            // Buttons row
            clearOledButton = new Button
            {
                Text = "Clear",
                Size = new Size(90, 28),
                Location = new Point(12, 26)
            };
            clearOledButton.Click += (s, e) =>
            {
                ClearScreen();
                oledPanel.Invalidate();
            };
            oledGroup.Controls.Add(clearOledButton);

            copyCArrayButton = new Button
            {
                Text = "Copy C Array",
                Size = new Size(120, 28),
                Location = new Point(clearOledButton.Right + 10, 26)
            };
            copyCArrayButton.Click += (s, e) =>
            {
                string c = ExportAsCArray("oled_bitmap_60x32");
                Clipboard.SetText(c);
                MessageBox.Show("Copied 240-byte page-major C array to clipboard.");
            };
            oledGroup.Controls.Add(copyCArrayButton);

            saveCArrayButton = new Button
            {
                Text = "Save .h",
                Size = new Size(90, 28),
                Location = new Point(copyCArrayButton.Right + 10, 26)
            };
            saveCArrayButton.Click += (s, e) => SaveCArrayToFile();
            oledGroup.Controls.Add(saveCArrayButton);

            editModeCheck = new CheckBox
            {
                Text = "Edit Mode (click to toggle pixels)",
                AutoSize = true,
                Location = new Point(saveCArrayButton.Right + 14, 30)
            };
            editModeCheck.CheckedChanged += (s, e) =>
            {
                if (!editModeCheck.Checked)
                    RenderAll();
            };
            oledGroup.Controls.Add(editModeCheck);

            // OLED drawing panel
            oledPanel = new Panel
            {
                Location = new Point(12, clearOledButton.Bottom + 10),
                Size = new Size(oledPanelW, oledPanelH),
                BackColor = Color.Black
            };
            oledPanel.Paint += OledPanel_Paint;
            oledPanel.MouseDown += OledPanel_MouseDown;
            oledGroup.Controls.Add(oledPanel);

            var note = new Label
            {
                Text = "Pages: 0-1 arrow | page 2 distance+unit | page 3 street\nExport: SSD1315/SSD1306 page-major (4 pages × 60 bytes = 240 bytes)",
                AutoSize = true,
                Location = new Point(12, oledPanel.Bottom + 10)
            };
            oledGroup.Controls.Add(note);

            // Wire render updates (only when not in edit mode)
            directionCombo.SelectedIndexChanged += AnyInputChanged;
            unitCombo.SelectedIndexChanged += AnyInputChanged;
            distanceTextbox.TextChanged += AnyInputChanged;
            streetTextbox.TextChanged += AnyInputChanged;
            specialCombo.SelectedIndexChanged += AnyInputChanged;

            void AnyInputChanged(object sender, EventArgs e)
            {
                if (!editModeCheck.Checked) RenderAll();
                UpdatePacketPreview();
            }

            // Window sizing
            int neededW = oledGroup.Right + pad;
            int neededH = Math.Max(oledGroup.Bottom + pad, packetGroup.Bottom + pad);
            ClientSize = new Size(neededW, Math.Max(760, neededH));
        }

        private static Label MkLabel(string text, int x, int y, int w)
        {
            return new Label
            {
                Text = text,
                AutoSize = false,
                TextAlign = ContentAlignment.MiddleRight,
                Location = new Point(x, y - 2),
                Size = new Size(w, 25)
            };
        }

        // ===================== Rendering Layout =====================
        private void RenderAll()
        {
            ClearScreen();

            bool left = (directionCombo.SelectedIndex == 0);
            DrawTurnArrow(left);

            RenderDistanceLine();
            RenderStreetLine();

            oledPanel.Invalidate();
        }

        private void RenderDistanceLine()
        {
            int y = 16; // page 2 start (y=16..23)

            // Build distance string for DISPLAY: up to 5 digits (dot doesn't count)
            string raw = (distanceTextbox.Text ?? "").Trim();

            int digitCount = 0;
            bool usedDot = false;
            var sb = new StringBuilder();

            foreach (char c in raw)
            {
                if (char.IsDigit(c))
                {
                    if (digitCount >= 5) break;
                    sb.Append(c);
                    digitCount++;
                }
                else if (c == '.' && !usedDot)
                {
                    if (digitCount > 0 && digitCount < 5)
                    {
                        sb.Append('.');
                        usedDot = true;
                    }
                }
            }

            string dist = sb.ToString();
            if (dist.Length == 0) dist = "0";

            string unit = (unitCombo.SelectedItem != null) ? unitCombo.SelectedItem.ToString() : "m";
            string line = dist + unit;

            DrawString6x8Centered(line, y, maxChars: 10);
        }

        private void RenderStreetLine()
        {
            int y = 24; // page 3 start (y=24..31)
            string street = (streetTextbox.Text ?? "");
            DrawString6x8Centered(street, y, maxChars: 8);
        }

        private void DrawString6x8Centered(string text, int y0, int maxChars)
        {
            if (text == null) text = "";
            if (text.Length > maxChars) text = text.Substring(0, maxChars);

            int charCount = text.Length;
            int textW = charCount * 6;
            int x0 = (OLED_W - textW) / 2;
            if (x0 < 0) x0 = 0;

            DrawString6x8(text, x0, y0, maxChars);
        }

        private void DrawString6x8(string text, int x0, int y0, int maxChars)
        {
            int x = x0;
            foreach (char ch in (text ?? "").Take(maxChars))
            {
                if (x + 5 >= OLED_W) break;
                DrawChar6x8(ch, x, y0);
                x += 6;
            }
        }

        // ===================== Arrow Drawing =====================
        private void DrawTurnArrow(bool left)
        {
            const int headW = 12;
            const int headH = 16; // pages 0–1

            int x0 = (OLED_W - headW) / 2; // centered horizontally
            int y0 = 0;                    // pages 0–1, symmetric in Y

            byte[] head = left ? ARROW_HEAD_LEFT_12x16 : ARROW_HEAD_RIGHT_12x16;
            BlitPageMajorBitmap(head, headW, headH, x0, y0);
        }





        private void FillRect(int x, int y, int w, int h)
        {
            if (w <= 0 || h <= 0) return;
            for (int yy = y; yy < y + h; yy++)
                for (int xx = x; xx < x + w; xx++)
                    SetPixel(xx, yy, true);
        }

        private void DrawVLine(int x, int y0, int y1)
        {
            if (y1 < y0) { int t = y0; y0 = y1; y1 = t; }
            for (int y = y0; y <= y1; y++)
                SetPixel(x, y, true);
        }

        private void SetPixel(int x, int y, bool on)
        {
            if (x < 0 || x >= OLED_W || y < 0 || y >= OLED_H) return;
            pixels[x, y] = on;
        }

        // ===================== OLED Panel Drawing / Editing =====================
        private void OledPanel_Paint(object sender, PaintEventArgs e)
        {
            Graphics g = e.Graphics;
            g.SmoothingMode = SmoothingMode.None;
            g.InterpolationMode = InterpolationMode.NearestNeighbor;
            g.PixelOffsetMode = PixelOffsetMode.Half;

            g.Clear(Color.Black);

            using (Brush onBrush = new SolidBrush(Color.White))
            using (Pen gridPen = new Pen(Color.FromArgb(35, 255, 255, 255), GRID_LINE_THICKNESS))
            {
                for (int py = 0; py < OLED_H; py++)
                {
                    for (int px = 0; px < OLED_W; px++)
                    {
                        if (!pixels[px, py]) continue;
                        g.FillRectangle(onBrush, px * SCALE, py * SCALE, SCALE, SCALE);
                    }
                }

                for (int x = 0; x <= OLED_W; x++)
                    g.DrawLine(gridPen, x * SCALE, 0, x * SCALE, OLED_H * SCALE);

                for (int y = 0; y <= OLED_H; y++)
                    g.DrawLine(gridPen, 0, y * SCALE, OLED_W * SCALE, y * SCALE);
            }
        }

        private void OledPanel_MouseDown(object sender, MouseEventArgs e)
        {
            if (!editModeCheck.Checked) return;

            int px = e.X / SCALE;
            int py = e.Y / SCALE;

            if (px < 0 || px >= OLED_W || py < 0 || py >= OLED_H)
                return;

            if (e.Button == MouseButtons.Left) pixels[px, py] = !pixels[px, py];
            else if (e.Button == MouseButtons.Right) pixels[px, py] = false;
            else if (e.Button == MouseButtons.Middle) pixels[px, py] = true;

            oledPanel.Invalidate();
        }

        private void ClearScreen()
        {
            for (int y = 0; y < OLED_H; y++)
                for (int x = 0; x < OLED_W; x++)
                    pixels[x, y] = false;
        }

        // ===================== 6x8 Font Rendering =====================
        private void DrawChar6x8(char ch, int x0, int y0)
        {
            if (y0 < 0 || y0 + 7 >= OLED_H) return;

            if (!Font6x8.TryGetValue(ch, out var cols))
            {
                char up = char.ToUpperInvariant(ch);
                if (!Font6x8.TryGetValue(up, out cols))
                    cols = Font6x8[' '];
            }

            for (int cx = 0; cx < 6; cx++)
            {
                int x = x0 + cx;
                if (x < 0 || x >= OLED_W) continue;

                byte col = cols[cx];
                for (int ry = 0; ry < 8; ry++)
                {
                    int y = y0 + ry;
                    bool on = ((col >> ry) & 0x01) != 0;
                    if (y >= 0 && y < OLED_H)
                        pixels[x, y] = on;
                }
            }
        }

        // ===================== UART Packet Build / Preview / Send =====================
        private void UpdatePacketPreview()
        {
            byte[] pkt = BuildNavPacket14();

            for (int i = 0; i < 14; i++)
            {
                if (packetByteBoxes[i] != null)
                    packetByteBoxes[i].Text = pkt[i].ToString() + " (0x" + pkt[i].ToString("X2") + ")";
            }
        }

        // ====== REPLACE your BuildNavPacket14() and ParseDistanceForEncoding() with the versions below,
        // and ADD the two new helpers ParseMeters0to9999() + ParseKmParts() anywhere in the Form1 class. ======

        // ===================== UART Packet Build / Preview / Send =====================
        private byte[] BuildNavPacket14()
        {
            byte[] pkt = new byte[14];

            // Byte 1
            pkt[0] = START_BYTE;

            // Byte 2: direction
            bool isLeft = (directionCombo.SelectedIndex == 0);
            pkt[1] = isLeft ? DIR_LEFT : DIR_RIGHT;

            // Unit (we need it before encoding distance)
            string unitStr = (unitCombo.SelectedItem != null) ? unitCombo.SelectedItem.ToString() : "m";
            bool isKm = (unitStr == "km");

            // Bytes 3-4: distance (UNIT-AWARE encoding!)
            //  - meters: 0..9999 encoded as [thousands+hundreds][tens+ones] where each byte is 0..99
            //  - km:     dist1 = integer km part (0..254), dist2 = decimals:
            //              dist2<10 => ".x"  (e.g., 5 => .5)
            //              dist2>=10 => ".xy" (e.g., 55 => .55)
            if (!isKm)
            {
                int meters = ParseMeters0to9999(distanceTextbox.Text); // ignore dot
                pkt[2] = (byte)(meters / 100);  // 00..99 (thousands+hundreds)
                pkt[3] = (byte)(meters % 100);  // 00..99 (tens+ones)
            }
            else
            {
                ParseKmParts(distanceTextbox.Text, out int kmInt, out int kmDec);

                if (kmInt < 0) kmInt = 0;
                if (kmInt > 254) kmInt = 254;   // reserve 255 as START_BYTE
                if (kmDec < 0) kmDec = 0;
                if (kmDec > 99) kmDec = 99;

                pkt[2] = (byte)kmInt;
                pkt[3] = (byte)kmDec;
            }

            // Byte 5: unit
            pkt[4] = isKm ? UNIT_KM : UNIT_M;

            // Bytes 6-13: street codes matching ER asc2_0806 indexing (chr - ' ')
            // => send only printable ASCII 0x20..0x7E. Anything else becomes space.
            string street = streetTextbox.Text ?? "";
            if (street.Length > 8) street = street.Substring(0, 8);
            street = street.PadRight(8, ' ');

            for (int i = 0; i < 8; i++)
            {
                char c = street[i];

                // Force into the same printable ASCII window the firmware font table supports
                if (c < ' ' || c > '~')
                    c = ' ';

                pkt[5 + i] = (byte)c;  // 0x20..0x7E
            }


            // Byte 14: special
            pkt[13] = EncodeSpecial();

            return pkt;
        }

        // OPTIONAL: You can delete ParseDistanceForEncoding() now (no longer used).
        // If you want to keep it, just don't call it anywhere.

        // ====== ADD THESE TWO HELPERS ======
        private int ParseMeters0to9999(string s)
        {
            if (s == null) return 0;

            // digits only, ignore '.', up to 4 digits (last 4 if longer)
            string digits = new string(s.Where(char.IsDigit).ToArray());
            if (digits.Length == 0) return 0;

            if (digits.Length > 4)
                digits = digits.Substring(digits.Length - 4, 4);

            if (!int.TryParse(digits, out int val)) val = 0;
            if (val < 0) val = 0;
            if (val > 9999) val = 9999;
            return val;
        }

        // For km input: "2.5" -> kmInt=2, kmDec=5
        //               "12.55" -> kmInt=12, kmDec=55
        //               "0.25" -> kmInt=0, kmDec=25
        //               "3" -> kmInt=3, kmDec=0
        private void ParseKmParts(string s, out int kmInt, out int kmDec)
        {
            kmInt = 0;
            kmDec = 0;

            if (string.IsNullOrWhiteSpace(s))
                return;

            s = s.Trim();

            int dot = s.IndexOf('.');
            string intPart = (dot >= 0) ? s.Substring(0, dot) : s;
            string fracPart = (dot >= 0) ? s.Substring(dot + 1) : "";

            // digits only
            intPart = new string(intPart.Where(char.IsDigit).ToArray());
            fracPart = new string(fracPart.Where(char.IsDigit).ToArray());

            if (intPart.Length > 0 && int.TryParse(intPart, out int tmpInt))
                kmInt = tmpInt;

            if (fracPart.Length == 0)
            {
                kmDec = 0;
            }
            else if (fracPart.Length == 1)
            {
                // "2.5" => 5 (special case dist2<10 means ".x")
                int d0 = fracPart[0] - '0';
                if (d0 < 0 || d0 > 9) d0 = 0;
                kmDec = d0;
            }
            else
            {
                // take first two digits: "12.55" => 55
                int d0 = fracPart[0] - '0';
                int d1 = fracPart[1] - '0';
                if (d0 < 0 || d0 > 9) d0 = 0;
                if (d1 < 0 || d1 > 9) d1 = 0;
                kmDec = d0 * 10 + d1;
            }
        }


        private int ParseDistanceForEncoding()
        {
            // For the 2-byte digit-chunk scheme described, distance is effectively 0..9999
            // We'll:
            //  - keep digits only
            //  - take up to 4 digits (thousands..ones)
            //  - if more digits entered, keep the last 4 (common UX), OR clamp; here we keep last 4.
            string raw = (distanceTextbox.Text ?? "").Trim();
            var digits = new string(raw.Where(char.IsDigit).ToArray());

            if (digits.Length == 0) return 0;

            if (digits.Length > 4)
                digits = digits.Substring(digits.Length - 4, 4);

            int val;
            if (!int.TryParse(digits, out val)) val = 0;
            if (val < 0) val = 0;
            if (val > 9999) val = 9999;
            return val;
        }

        private byte EncodeSpecial()
        {
            if (specialCombo.SelectedIndex <= 0) return SPECIAL_NONE;
            if (specialCombo.SelectedIndex == 1) return SPECIAL_WRONG_TURN;
            if (specialCombo.SelectedIndex == 2) return SPECIAL_ACCIDENT;
            return SPECIAL_NONE;
        }

        private void SendNavPacket()
        {
            byte[] pkt = BuildNavPacket14();
            UpdatePacketPreview();

            if (mySerialPort == null || !mySerialPort.IsOpen)
            {
                MessageBox.Show("Serial port is not connected.");
                return;
            }

            try
            {
                mySerialPort.Write(pkt, 0, pkt.Length);
            }
            catch (Exception ex)
            {
                MessageBox.Show("UART send failed: " + ex.Message);
            }
        }

        // ===================== Export (SSD1315/SSD1306 page-major) =====================
        private byte[] ExportPageMajor240()
        {
            int pages = OLED_H / 8; // 4
            byte[] data = new byte[OLED_W * pages]; // 240

            for (int page = 0; page < pages; page++)
            {
                for (int x = 0; x < OLED_W; x++)
                {
                    byte b = 0;
                    for (int bit = 0; bit < 8; bit++)
                    {
                        int y = page * 8 + bit;
                        if (pixels[x, y])
                            b |= (byte)(1 << bit);
                    }
                    data[page * OLED_W + x] = b;
                }
            }
            return data;
        }

        private string ExportAsCArray(string name)
        {
            byte[] bytes = ExportPageMajor240();

            var lines = new List<string>();
            for (int i = 0; i < bytes.Length; i += 12)
            {
                var chunk = bytes.Skip(i).Take(12).Select(b => "0x" + b.ToString("X2")).ToArray();
                lines.Add("  " + string.Join(", ", chunk));
            }

            return
$@"// 60x32 OLED bitmap, SSD1315/SSD1306 page-major format
// 4 pages (8 rows each) × 60 columns = 240 bytes
const unsigned char {name}[240] = {{
{string.Join(",\n", lines)}
}};";
        }

        private void SaveCArrayToFile()
        {
            using (SaveFileDialog sfd = new SaveFileDialog())
            {
                sfd.Filter = "Header file (*.h)|*.h|C file (*.c)|*.c|Text file (*.txt)|*.txt";
                sfd.Title = "Save OLED Bitmap C Array";
                sfd.FileName = "oled_bitmap_60x32.h";

                if (sfd.ShowDialog() == DialogResult.OK)
                {
                    try
                    {
                        File.WriteAllText(sfd.FileName, ExportAsCArray("oled_bitmap_60x32"));
                        MessageBox.Show("Saved.");
                    }
                    catch (Exception ex)
                    {
                        MessageBox.Show("Save failed: " + ex.Message);
                    }
                }
            }
        }

        // ===================== Serial Connect/Disconnect =====================
        private void connectCOM(object sender, EventArgs e)
        {
            if (!mySerialPort.IsOpen)
            {
                try
                {
                    mySerialPort.PortName = comselect.Text;
                    mySerialPort.Open();
                    comconnect.Text = "Disconnect";
                }
                catch
                {
                    MessageBox.Show("Failed to open port.");
                }
            }
            else
            {
                try { mySerialPort.Close(); } catch { }
                comconnect.Text = "Connect";
            }
        }

        private void BlitPageMajorBitmap(byte[] data, int bmpW, int bmpH, int x0, int y0)
        {
            int pages = bmpH / 8; // for 16px height => 2 pages
            for (int page = 0; page < pages; page++)
            {
                for (int x = 0; x < bmpW; x++)
                {
                    byte b = data[page * bmpW + x];
                    for (int bit = 0; bit < 8; bit++)
                    {
                        int px = x0 + x;
                        int py = y0 + page * 8 + bit;

                        bool on = ((b >> bit) & 0x01) != 0;
                        if (on) SetPixel(px, py, true);
                    }
                }
            }
        }


        // ===================== Font Table (subset) =====================
        private static Dictionary<char, byte[]> BuildFont6x8()
        {
            var f = new Dictionary<char, byte[]>();

            void Add(char c, params byte[] cols)
            {
                if (cols.Length != 6) throw new ArgumentException("6 columns required.");
                f[c] = cols;
            }

            // Space + punctuation
            Add(' ', 0x00, 0x00, 0x00, 0x00, 0x00, 0x00);
            Add('-', 0x00, 0x08, 0x08, 0x08, 0x08, 0x08);
            Add('.', 0x00, 0x00, 0x60, 0x60, 0x00, 0x00);
            Add('/', 0x00, 0x20, 0x10, 0x08, 0x04, 0x02);

            // Digits
            Add('0', 0x00, 0x3E, 0x51, 0x49, 0x45, 0x3E);
            Add('1', 0x00, 0x00, 0x42, 0x7F, 0x40, 0x00);
            Add('2', 0x00, 0x42, 0x61, 0x51, 0x49, 0x46);
            Add('3', 0x00, 0x21, 0x41, 0x45, 0x4B, 0x31);
            Add('4', 0x00, 0x18, 0x14, 0x12, 0x7F, 0x10);
            Add('5', 0x00, 0x27, 0x45, 0x45, 0x45, 0x39);
            Add('6', 0x00, 0x3C, 0x4A, 0x49, 0x49, 0x30);
            Add('7', 0x00, 0x01, 0x71, 0x09, 0x05, 0x03);
            Add('8', 0x00, 0x36, 0x49, 0x49, 0x49, 0x36);
            Add('9', 0x00, 0x06, 0x49, 0x49, 0x29, 0x1E);

            // Uppercase A-Z (subset used for streets)
            Add('A', 0x00, 0x7C, 0x12, 0x11, 0x12, 0x7C);
            Add('B', 0x00, 0x7F, 0x49, 0x49, 0x49, 0x36);
            Add('C', 0x00, 0x3E, 0x41, 0x41, 0x41, 0x22);
            Add('D', 0x00, 0x7F, 0x41, 0x41, 0x22, 0x1C);
            Add('E', 0x00, 0x7F, 0x49, 0x49, 0x49, 0x41);
            Add('F', 0x00, 0x7F, 0x09, 0x09, 0x09, 0x01);
            Add('G', 0x00, 0x3E, 0x41, 0x49, 0x49, 0x7A);
            Add('H', 0x00, 0x7F, 0x08, 0x08, 0x08, 0x7F);
            Add('I', 0x00, 0x00, 0x41, 0x7F, 0x41, 0x00);
            Add('J', 0x00, 0x20, 0x40, 0x41, 0x3F, 0x01);
            Add('K', 0x00, 0x7F, 0x08, 0x14, 0x22, 0x41);
            Add('L', 0x00, 0x7F, 0x40, 0x40, 0x40, 0x40);
            Add('M', 0x00, 0x7F, 0x02, 0x0C, 0x02, 0x7F);
            Add('N', 0x00, 0x7F, 0x04, 0x08, 0x10, 0x7F);
            Add('O', 0x00, 0x3E, 0x41, 0x41, 0x41, 0x3E);
            Add('P', 0x00, 0x7F, 0x09, 0x09, 0x09, 0x06);
            Add('Q', 0x00, 0x3E, 0x41, 0x51, 0x21, 0x5E);
            Add('R', 0x00, 0x7F, 0x09, 0x19, 0x29, 0x46);
            Add('S', 0x00, 0x46, 0x49, 0x49, 0x49, 0x31);
            Add('T', 0x00, 0x01, 0x01, 0x7F, 0x01, 0x01);
            Add('U', 0x00, 0x3F, 0x40, 0x40, 0x40, 0x3F);
            Add('V', 0x00, 0x1F, 0x20, 0x40, 0x20, 0x1F);
            Add('W', 0x00, 0x3F, 0x40, 0x38, 0x40, 0x3F);
            Add('X', 0x00, 0x63, 0x14, 0x08, 0x14, 0x63);
            Add('Y', 0x00, 0x07, 0x08, 0x70, 0x08, 0x07);
            Add('Z', 0x00, 0x61, 0x51, 0x49, 0x45, 0x43);

            // Lowercase a-z
            Add('a', 0x00, 0x20, 0x54, 0x54, 0x54, 0x78);
            Add('b', 0x00, 0x7F, 0x48, 0x44, 0x44, 0x38);
            Add('c', 0x00, 0x38, 0x44, 0x44, 0x44, 0x20);
            Add('d', 0x00, 0x38, 0x44, 0x44, 0x48, 0x7F);
            Add('e', 0x00, 0x38, 0x54, 0x54, 0x54, 0x18);
            Add('f', 0x00, 0x08, 0x7E, 0x09, 0x01, 0x02);
            Add('g', 0x00, 0x18, 0xA4, 0xA4, 0xA4, 0x7C);
            Add('h', 0x00, 0x7F, 0x08, 0x04, 0x04, 0x78);
            Add('i', 0x00, 0x00, 0x44, 0x7D, 0x40, 0x00);
            Add('j', 0x00, 0x40, 0x80, 0x84, 0x7D, 0x00);
            Add('k', 0x00, 0x7F, 0x10, 0x28, 0x44, 0x00);
            Add('l', 0x00, 0x00, 0x41, 0x7F, 0x40, 0x00);
            Add('m', 0x00, 0x7C, 0x04, 0x18, 0x04, 0x78);
            Add('n', 0x00, 0x7C, 0x08, 0x04, 0x04, 0x78);
            Add('o', 0x00, 0x38, 0x44, 0x44, 0x44, 0x38);
            Add('p', 0x00, 0xFC, 0x24, 0x24, 0x24, 0x18);
            Add('q', 0x00, 0x18, 0x24, 0x24, 0x18, 0xFC);
            Add('r', 0x00, 0x7C, 0x08, 0x04, 0x04, 0x08);
            Add('s', 0x00, 0x48, 0x54, 0x54, 0x54, 0x20);
            Add('t', 0x00, 0x04, 0x3F, 0x44, 0x40, 0x20);
            Add('u', 0x00, 0x3C, 0x40, 0x40, 0x20, 0x7C);
            Add('v', 0x00, 0x1C, 0x20, 0x40, 0x20, 0x1C);
            Add('w', 0x00, 0x3C, 0x40, 0x30, 0x40, 0x3C);
            Add('x', 0x00, 0x44, 0x28, 0x10, 0x28, 0x44);
            Add('y', 0x00, 0x1C, 0xA0, 0xA0, 0xA0, 0x7C);
            Add('z', 0x00, 0x44, 0x64, 0x54, 0x4C, 0x44);

            return f;
        }

        // 28x16 arrow icon in SSD1306/SSD1315 page-major format:
        // first 28 bytes = page 0 (rows 0..7), next 28 bytes = page 1 (rows 8..15)
        // 28x16 arrow icon in SSD1306/SSD1315 page-major format:
        // first 28 bytes = page 0 (rows 0..7), next 28 bytes = page 1 (rows 8..15)

        static readonly byte[] ARROW_HEAD_RIGHT_12x16 = new byte[]
        {
    0x08, 0x14, 0x24, 0x48, 0x88, 0x10, 0x10, 0x20, 0x20, 0x40, 0x40, 0x80,
    0x08, 0x14, 0x12, 0x09, 0x08, 0x04, 0x04, 0x02, 0x02, 0x01, 0x01, 0x00
        };

        static readonly byte[] ARROW_HEAD_LEFT_12x16 = new byte[]
        {
    0x80, 0x40, 0x40, 0x20, 0x20, 0x10, 0x10, 0x88, 0x48, 0x24, 0x14, 0x08,
    0x00, 0x01, 0x01, 0x02, 0x02, 0x04, 0x04, 0x08, 0x09, 0x12, 0x14, 0x08
        };



    }
}
