demoUpdated.py
	Final working version. Generates a turn by turn route from GPS coordinates, and updates the microOLED display on the glasses at the click of a button. The RN_ADDR must be updated to match the device specific address of the module, see the description of checkUUID.py below.

rn4871_ble.py
	A small BLE driver. Runs Bleak/asyncio on a dedicated background thread. Is called from demoUpdated.py, so it must be in the same folder (or the path can be specified in import).

checkUUID.py
	Used to discover the device specific address of the RN4871 module.

test_route.py
	Used to validate that Valhalla is running correctly through Docker.