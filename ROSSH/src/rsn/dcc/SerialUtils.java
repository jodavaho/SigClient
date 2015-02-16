package rsn.dcc;

import gnu.io.CommPortIdentifier;
import gnu.io.NoSuchPortException;
import gnu.io.PortInUseException;
import gnu.io.SerialPort;
import gnu.io.UnsupportedCommOperationException;

import java.io.IOException;
import java.io.InputStream;
import java.io.OutputStream;
import java.util.Vector;

public class SerialUtils {

	public static final int DEFAULT_PORT_SPEED = 57600;
	public static final int EXIT_SUCCESS = 0;
	public static final int EXIT_FAILURE = 1;
	public static final int EOFREACHED_WAIT_MS = 50;
	public static final boolean verbose = false;

	/**
	 * Opens a serial port with given serial device and baud rate
	 * 
	 * @param portId
	 *            port Id
	 * @param serialPort
	 *            SerialPort object
	 * @param serialDevice
	 *            path and name of the serial device
	 * @return EXIT_FAILURE or EXIT_SUCCESS
	 */
	public static SerialPort openPort(String serialDevice) {

		SerialPort serialPort = null;
		CommPortIdentifier portId = null;
		//
		// Open the serial port.
		//
		try {
			portId = CommPortIdentifier.getPortIdentifier(serialDevice);
		} catch (NoSuchPortException e1) {
			System.err.println("Error: Could not open serial port "
					+ serialDevice);
			e1.printStackTrace();
			System.exit(1);
		}

		try {
			serialPort = (SerialPort) portId.open("Cyclopse", 2000);
		} catch (PortInUseException e) {
			System.err.println("Error: Could not open serial port "
					+ serialDevice);
			e.printStackTrace();
			System.exit(1);
		}

		serialPort.notifyOnDataAvailable(true);
		try {
			serialPort.setSerialPortParams(DEFAULT_PORT_SPEED,
					SerialPort.DATABITS_8, SerialPort.STOPBITS_1,
					SerialPort.PARITY_NONE);

			serialPort.setFlowControlMode(SerialPort.FLOWCONTROL_NONE);

			// serialPort.enableReceiveTimeout(100);
		} catch (UnsupportedCommOperationException e) {
			System.err.println("Error: Could not open serial port "
					+ serialDevice);
			e.printStackTrace();
			System.exit(1);
		}
		return serialPort;
	}

	/**
	 * This method allows the user to set parameters of the open
	 * 
	 * @param serialDevice
	 *            path to the serial port
	 * @param baud
	 *            baud rate
	 * @param waitTime
	 *            maximum wait time for a signal
	 */
	public static SerialPort openPort(String serialDevice, int baud,
			int waitTime) {

		SerialPort serialPort = null;
		CommPortIdentifier portId = null;
		//
		// Open the serial port.
		//
		try {
			portId = CommPortIdentifier.getPortIdentifier(serialDevice);
		} catch (NoSuchPortException e1) {
			System.err.println("Error: Could not open serial port "
					+ serialDevice);
			e1.printStackTrace();
			System.exit(1);
		}

		try {
			serialPort = (SerialPort) portId.open("DCC", waitTime);
		} catch (PortInUseException e) {
			System.err.println("Error: Could not open serial port "
					+ serialDevice);
			e.printStackTrace();
			System.exit(1);
		}

		serialPort.notifyOnDataAvailable(true);
		try {
			serialPort.setSerialPortParams(baud, SerialPort.DATABITS_8,
					SerialPort.STOPBITS_1, SerialPort.PARITY_NONE);

			serialPort.setFlowControlMode(SerialPort.FLOWCONTROL_NONE);

			// serialPort.enableReceiveTimeout(100);
		} catch (UnsupportedCommOperationException e) {
			System.err.println("Error: Could not open serial port "
					+ serialDevice);
			e.printStackTrace();
			System.exit(1);
		}
		return serialPort;
	}

	/**
	 * Writes an array of characters to serial port
	 * 
	 * @param input
	 *            an array of input characters
	 * @param serialPort
	 *            the serial port object the bytes will be written
	 * @return true if successful
	 */
	public static boolean writePort(Vector<Character> input,
			SerialPort serialPort) {

		// openPort();
		//
		// Write the character array to serial port
		//
		OutputStream outputStream;
		if (verbose)
			System.out.println("Writing command to serial port.");
		try {
			outputStream = serialPort.getOutputStream();
			for (int i = 0; i < input.size(); i++) {
				char next_byte = input.elementAt(i);
				if (verbose)
					System.out.print(Integer.toHexString(next_byte) + " ");
				try {
					outputStream.write(next_byte);
				} catch (IOException e) {
					System.err.println("Could not write to output stream");
					e.printStackTrace();
					System.exit(1);
				}
			}

			if (verbose)
				System.out.println();
			if (verbose)
				System.out.println("Done.");
			try {
				Thread.sleep(50); // Be sure data is xferred before closing
			} catch (Exception e) {
			}
			// serialPort.close();
		} catch (IOException e) {
			System.err.println("Could not get the output stream");
			e.printStackTrace();
			System.exit(1);
		}

		return true;
	}

	/**
	 * Writes command (requested packetID command) and reads an array of
	 * characters from serial port
	 * 
	 * @param input
	 *            an array of characters as command
	 * @param numOfBytes
	 *            an integer value which represents the number of bytes read
	 *            from serial port
	 * @param serialPort
	 *            SerialPort object
	 * @return output an array of output characters
	 * @return true if successful
	 */
	public static boolean writeReadPort(Vector<Character> input,
			Vector<Character> output, int numOfBytes, SerialPort serialPort) {

		// openPort();
		//
		// Write the character array to serial port
		//
		InputStream inputStream;
		OutputStream outputStream;

		try {
			inputStream = serialPort.getInputStream();
			outputStream = serialPort.getOutputStream();

			//
			// Flush input stream by reading in all available bytes
			//
			byte[] flushBuffer = new byte[1024];
			int readCount = inputStream.available();
			int m=0;
			int readVal=0;
			for( m=1; m< readCount/1024;m++){

				readVal += inputStream.read(flushBuffer, 0, 1024);
			}	
//			inputStream.read(flushBuffer, 0, readCount - readVal);
			inputStream.read(flushBuffer, 0, inputStream.available());

			if (verbose)
				System.out.println("Writing command to serial port.");

			for (int i = 0; i < input.size(); i++) {
				char next_byte = input.elementAt(i);
				if (verbose)
					System.out.print(Integer.toHexString(next_byte) + " ");
				try {
					outputStream.write(next_byte);
				} catch (IOException e) {
					System.err.println("Could not write to output stream");
					e.printStackTrace();
					System.exit(1);
				}
			}

			if (verbose)
				System.out.println();
			if (verbose)
				System.out.println("Done.");

			//
			// Keep reading data from serial port and print it to the screen.
			//
			if (verbose)
				System.out.println("Reading command from serial port.");

			byte[] readBuffer = new byte[128];

			try {
				for (int i = 0; i < numOfBytes;) {

					int numBytesRead = inputStream.read(readBuffer);

					i += numBytesRead;
					for (int j = 0; j < numBytesRead; j++) {
						char next_byte = (char) readBuffer[j];
						next_byte &= 0x00ff;
						output.add(next_byte);
						if (verbose)
							System.out.print(Integer.toHexString(next_byte)
									+ " ");
					}
				}

			} catch (IOException e) {
				System.err.println("Could not read from input stream");
				e.printStackTrace();
				System.exit(1);
			}
			if (verbose)
				System.out.println();
			if (verbose)
				System.out.println("Done.");

			// serialPort.close();
		} catch (IOException e) {
			System.err.println("Could not get the input/output stream");
			e.printStackTrace();
			System.exit(1);
		}catch(IndexOutOfBoundsException e){
			System.err.println("Indexoutofbound exception");
			serialPort.close();
			serialPort = openPort("/dev/ttyUSB1"); 	
			writeReadPort(input,
					output,  numOfBytes,  serialPort);
		}
		return true;
	}

	/**
	 * Writes @param input to the @param serialPort and then waits a specified
	 * time, @param waitTime, before it returns true
	 */
	public static boolean writePortDelay(Vector<Character> input,
			SerialPort serialPort, int waitTime) {

		// openPort();
		//
		// Write the character array to serial port
		//
		OutputStream outputStream;
		if (verbose)
			System.out.println("Writing command to serial port.");
		try {
			outputStream = serialPort.getOutputStream();
			for (int i = 0; i < input.size(); i++) {
				char next_byte = input.elementAt(i);
				if (verbose)
					System.out.print(Integer.toHexString(next_byte) + " ");
				try {
					outputStream.write(next_byte);
				} catch (IOException e) {
					System.err.println("Could not write to output stream");
					e.printStackTrace();
					System.exit(1);
				}
			}
			if (waitTime != 0) {
				serialPort.sendBreak(waitTime);
			}

			if (verbose)
				System.out.println();
			if (verbose)
				System.out.println("Done.");
			try {
				Thread.sleep(50); // Be sure data is xferred before closing
			} catch (Exception e) {
			}
			// serialPort.close();
		} catch (IOException e) {
			System.err.println("Could not get the output stream");
			e.printStackTrace();
			System.exit(1);
		}

		return true;
	}


}
