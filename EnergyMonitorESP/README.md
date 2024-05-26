
## Error codes

Code | Description                                                     | Fix
-----|-----------------------------------------------------------------|------------------------------------------------------------------------------------------------
E11  | The server sync payload exceeded the expected size              | Software fault
E21  | The sensor recieved erratic data                                | This means the sensor is not recieving the expected 100ms pulses from the meter. Ensure that the sensor is attached to the meter and there is not excessive ambient light. Ensure the LED on the sensor is pulsing shortly periodically
E22  | Over-the-air (OTA) update failed                                 | Ensure WiFi is connected
E23  | Failed to open config file for reading                          | Software fault
E24  | Failed to open config file for writing                          | Software fault
E25  | Failed to write to config file                          | Software fault
E30  | Invalid config                          | Ensure config is correct
E31  | Invalid secret                          | Ensure secret is set
E32  | Invalid sensor id                          | Ensure sensor id is set
E33  | Invalid endpoint                          | Ensure endpoint is correct
E34  | Invalid sync route                          | Ensure syncRoute is correct
E35  | Invalid refresh route                      | Ensure refreshRoute is correct
E36  | Invalid refresh period                          | Ensure refresh period is an integer between 5 and 3600
E37  | Invalid impluses per kwh                          | Ensure impluses per kwh is a positive integer 
