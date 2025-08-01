# CircuitPython Library specifically for the TOFSense F2 sensor.
# This version is simplified to remove support for other models.

class TOFSenseF2:
    """
    A dedicated driver for the TOFSense F2 distance sensor.
    Handles UART communication and data parsing for the F2 model.
    """
    def __init__(self, uart):
        """
        Initializes the TOFSense F2 driver.
        :param uart: An initialized busio.UART object.
        """
        # Protocol constants specific to the TOFSense F2
        self._FRAME_HEADER = 0x57
        self._FRAME_MARK = 0x00
        self._DATA_LENGTH = 16
        
        self.uart = uart
        self.data_dict = {
            "id": 0,
            "system_time": 0,
            "dis": 0.0,
            "dis_status": 0,
            "signal_strength": 0,
            "range_precision": 0,
        }

    def _read_byte(self):
        """Reads a single byte from the UART bus."""
        byte = self.uart.read(1)
        return byte[0] if byte is not None else None

    def _get_data_frame(self):
        """
        Reads a complete data frame from the UART bus.
        """
        if self.uart is None:
            return None

        # Continuously look for the frame header
        while True:
            header_byte = self._read_byte()
            if header_byte is None: # Timeout
                return None
            if header_byte == self._FRAME_HEADER:
                # Header found, now look for the frame mark
                mark_byte = self._read_byte()
                if mark_byte == self._FRAME_MARK:
                    # Mark found, read the rest of the data payload
                    payload_len = self._DATA_LENGTH - 2
                    payload = self.uart.read(payload_len)
                    if payload and len(payload) == payload_len:
                        # Construct and return the full frame
                        full_frame = bytearray([self._FRAME_HEADER, self._FRAME_MARK])
                        full_frame.extend(payload)
                        return full_frame
                # If mark is not found, continue searching for the next header
        return None

    def _check_data(self, data):
        """
        Validates a data frame using a checksum.
        :param data: The bytearray data frame to check.
        :return: A list of hex values if valid, otherwise False.
        """
        if data is None or not isinstance(data, (bytes, bytearray)):
            return False

        # Extract data payload and the original checksum
        data_payload = data[:-1]
        original_checksum = data[-1]

        # Calculate the expected checksum
        calculated_checksum = sum(data_payload) & 0xFF

        if calculated_checksum != original_checksum:
            print(f"Checksum mismatch: Got {original_checksum}, calculated {calculated_checksum}")
            return False
        
        # Convert to a list of hex strings for easier parsing
        return [f"{byte:02x}" for byte in data]

    def _send_read_frame(self, sensor_id):
        """
        Sends a query frame to the sensor to request data.
        :param sensor_id: The ID of the sensor to query (0-255).
        """
        if self.uart is None:
            return
        try:
            # Protocol data for the query frame
            protocol_data = [0x57, 0x10, 0xFF, 0xFF, sensor_id, 0xFF, 0xFF]
            # Calculate and append the checksum
            checksum = sum(protocol_data) & 0xFF
            protocol_data.append(checksum)
            # Convert to bytes and send over UART
            self.uart.write(bytes(protocol_data))
            return True
        except Exception as e:
            print(f"An exception occurred while sending data: {e}")
            return None

    def get_data(self):
        """
        Gets and parses data from the sensor in active output mode.
        :return: A dictionary with parsed data, or None if an error occurs.
        """
        raw_data = self._get_data_frame()
        if raw_data:
            return self.__unpack_data(raw_data)
        return None

    def get_data_inquire(self, sensor_id=0):
        """
        Gets and parses data from the sensor in query mode.
        :param sensor_id: The ID of the sensor to query.
        :return: A dictionary with parsed data, or None if an error occurs.
        """
        self._send_read_frame(sensor_id)
        raw_data = self._get_data_frame()
        if raw_data:
            return self.__unpack_data(raw_data)
        return None

    def __unpack_data(self, data):
        """
        Parses a raw data frame into the data_dict.
        :param data: The raw bytearray data frame.
        """
        hex_list = self._check_data(data)
        if not hex_list:
            return None

        try:
            # Parse data fields according to the TOFSense protocol
            self.data_dict["id"] = int(hex_list[3], 16)
            self.data_dict["system_time"] = int(hex_list[7] + hex_list[6] + hex_list[5] + hex_list[4], 16)
            # Distance is a 3-byte little-endian value in mm, convert to meters
            self.data_dict["dis"] = int(hex_list[10] + hex_list[9] + hex_list[8], 16) / 1000.0
            self.data_dict["dis_status"] = int(hex_list[11], 16)
            self.data_dict["signal_strength"] = int(hex_list[13] + hex_list[12], 16)
            self.data_dict["range_precision"] = int(hex_list[14], 16)
            return self.data_dict
        except (ValueError, IndexError) as e:
            print(f"Error parsing data: {e}")
            return None
