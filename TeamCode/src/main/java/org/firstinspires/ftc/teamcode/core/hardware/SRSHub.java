package org.firstinspires.ftc.teamcode.core.hardware;

import static java.lang.Thread.sleep;

import androidx.annotation.NonNull;

import com.qualcomm.hardware.lynx.LynxI2cDeviceSynch;
import com.qualcomm.robotcore.hardware.I2cAddr;
import com.qualcomm.robotcore.hardware.I2cDeviceSynchDevice;
import com.qualcomm.robotcore.hardware.I2cDeviceSynchSimple;
import com.qualcomm.robotcore.hardware.configuration.annotations.DeviceProperties;
import com.qualcomm.robotcore.hardware.configuration.annotations.I2cDeviceType;
import com.qualcomm.robotcore.util.RobotLog;

import java.nio.ByteBuffer;
import java.nio.ByteOrder;
import java.util.ArrayList;
import java.util.Arrays;
import java.util.BitSet;

@I2cDeviceType
@DeviceProperties(xmlTag = "SRSHub", name = "SRSHub")
public class SRSHub extends I2cDeviceSynchDevice<I2cDeviceSynchSimple> {
    private static final int I2C_ADDRESS = 0x57;

    private static final int DEVICE_ID = 0x61;

    private static final int DEVICE_MAJOR_VERSION = 1;
    private static final int DEVICE_MINOR_VERSION = 4;
    private static final int DEVICE_PATCH_VERSION = 4;

    private static final int BITS_PER_ANALOG_DIGITAL_DEVICE = 2;
    private static final int BITS_PER_ENCODER = 2;
    private static final int MIN_BITS_PER_I2C_BUS = 16;
    private static final int BITS_PER_I2C_DEVICE = 4;

    private static final ByteOrder BYTE_ORDER = ByteOrder.LITTLE_ENDIAN;

    private Config config;

    private int updateLength = 8;

    private boolean ready = false;
    private boolean disconnected = false;

    /**
     * Number of updates dropped due to a CRC mismatch. A mismatch leaves
     * {@link #disconnected()} false while every cached value silently goes
     * stale, so callers should surface this in telemetry.
     */
    private long crcMismatchCount = 0;

    private final double[] analogDigitalValues =
        new double[12];

    private final PosVel[] encoderValues = new PosVel[6];

    public enum AnalogDigitalDevice {
        ANALOG(0),
        DIGITAL(1),
        NONE(2);

        final int value;

        AnalogDigitalDevice(int value) {
            this.value = value;
        }
    }

    public enum Encoder {
        QUADRATURE(0),
        PWM(1),
        NONE(2);

        final int value;

        Encoder(int value) {
            this.value = value;
        }
    }

    public static class PosVel {
        public int position = 0;
        public int velocity = 0;
    }

    public static abstract class Command {
        protected abstract int getLength();

        protected abstract BitSet getData();
    }

    public static abstract class I2CDevice {
        protected abstract int getValue();

        protected abstract int getInitLength();

        protected abstract int getUpdateLength();

        protected abstract int getAddress();

        protected abstract BitSet getConfig();

        protected abstract void parseUpdate(BitSet data, int start);
    }

    public static class APDS9151 extends I2CDevice {
        private final BitSet config = new BitSet(getInitLength());

        public boolean disconnected = false;

        public short proximity;
        public int infrared;
        public int red;
        public int green;
        public int blue;

        protected int getValue() {
            return 0;
        }

        protected int getInitLength() {
            return 0;
        }

        protected int getUpdateLength() {
            return 76;
        }

        protected int getAddress() {
            return 0x52;
        }

        protected BitSet getConfig() {
            return config;
        }

        protected void parseUpdate(BitSet data, int start) {
            int index = start;

            disconnected = data.get(index++);

            byte[] proximityChunk = data
                .get(
                    index,
                    index + 11
                )
                .toByteArray();

            byte[] paddedProximityChunk = new byte[2];

            System.arraycopy(
                proximityChunk,
                0,
                paddedProximityChunk,
                0,
                proximityChunk.length
            );

            proximity = ByteBuffer
                .wrap(paddedProximityChunk)
                .order(BYTE_ORDER)
                .getShort();

            index += 11;

            byte[] infraredChunk = data
                .get(
                    index,
                    index + 16
                )
                .toByteArray();

            byte[] paddedInfraredChunk = new byte[4];

            System.arraycopy(
                infraredChunk,
                0,
                paddedInfraredChunk,
                0,
                infraredChunk.length
            );

            infrared = ByteBuffer
                .wrap(paddedInfraredChunk)
                .order(BYTE_ORDER)
                .getInt();

            index += 16;

            byte[] redChunk = data
                .get(
                    index,
                    index + 16
                )
                .toByteArray();

            byte[] paddedRedChunk = new byte[4];

            System.arraycopy(
                redChunk,
                0,
                paddedRedChunk,
                0,
                redChunk.length
            );

            red = ByteBuffer
                .wrap(paddedRedChunk)
                .order(BYTE_ORDER)
                .getInt();

            index += 16;

            byte[] greenChunk = data
                .get(
                    index,
                    index + 16
                )
                .toByteArray();

            byte[] paddedGreenChunk = new byte[4];

            System.arraycopy(
                greenChunk,
                0,
                paddedGreenChunk,
                0,
                greenChunk.length
            );

            green = ByteBuffer
                .wrap(paddedGreenChunk)
                .order(BYTE_ORDER)
                .getInt();

            index += 16;

            byte[] blueChunk = data
                .get(
                    index,
                    index + 16
                )
                .toByteArray();

            byte[] paddedBlueChunk = new byte[4];

            System.arraycopy(
                blueChunk,
                0,
                paddedBlueChunk,
                0,
                blueChunk.length
            );

            blue = ByteBuffer
                .wrap(paddedBlueChunk)
                .order(BYTE_ORDER)
                .getInt();
        }
    }

    public static class VL53L5CX extends I2CDevice {
        public enum Resolution {
            GRID_4x4(0),
            GRID_8x8(1);

            final byte value;

            Resolution(int value) {
                this.value = (byte) value;
            }
        }

        private final BitSet config = new BitSet(getInitLength());

        public boolean disconnected = false;

        public final short[] distances;

        public VL53L5CX(Resolution resolution) {
            config.set(
                0,
                resolution.value == 1
            );

            distances = resolution.value == 0 ? new short[16] : new short[64];
        }

        protected int getValue() {
            return 1;
        }

        protected int getInitLength() {
            return 1;
        }

        protected int getUpdateLength() {
            return config.get(0) ? 705 : 177;
        }

        protected int getAddress() {
            return 0x29;
        }

        protected BitSet getConfig() {
            return config;
        }

        protected void parseUpdate(BitSet data, int start) {
            int index = start;

            disconnected = data.get(index++);

            for (int k = 0; k < (getUpdateLength() - 1) / 11; k++) {
                byte[] chunk = data
                    .get(
                        index,
                        index + 11
                    )
                    .toByteArray();

                index += 11;

                byte[] paddedChunk = new byte[2];

                System.arraycopy(
                    chunk,
                    0,
                    paddedChunk,
                    0,
                    chunk.length
                );

                distances[k] = ByteBuffer
                    .wrap(paddedChunk)
                    .order(BYTE_ORDER)
                    .getShort();
            }
        }
    }

    public static class VL53L0X extends I2CDevice {
        private final BitSet config = new BitSet(getInitLength());

        public boolean disconnected = false;

        public float distance;

        protected int getValue() {
            return 2;
        }

        protected int getInitLength() {
            return 0;
        }

        protected int getUpdateLength() {
            return 17;
        }

        protected int getAddress() {
            return 0x29;
        }

        protected BitSet getConfig() {
            return config;
        }

        protected void parseUpdate(BitSet data, int start) {
            int index = start;

            disconnected = data.get(index++);

            byte[] chunk = data
                .get(
                    index,
                    index + 16
                )
                .toByteArray();

            byte[] paddedChunk = new byte[4];

            System.arraycopy(
                chunk,
                0,
                paddedChunk,
                0,
                chunk.length
            );

            distance = ByteBuffer
                .wrap(paddedChunk)
                .order(BYTE_ORDER)
                .getShort() & 0xFFFF;
        }
    }

    public static class GoBildaPinpoint extends I2CDevice {
        private static final int ID = 3;

        public enum EncoderDirection {
            FORWARD,
            REVERSED
        }

        public static class ResetIMUCommand extends Command {
            private final BitSet data;

            /**
             * @param bus the bus of the command's target device
             */
            public ResetIMUCommand(int bus) {
                data = new BitSet(getLength());

                data.set(
                    0,
                    (bus & 1) == 1
                );

                data.set(
                    1,
                    (bus >> 1 & 1) == 1
                );

                int index = 2;

                for (int i = 0; i < BITS_PER_I2C_DEVICE; i++) {
                    data.set(
                        index++,
                        (ID >> i & 1) == 1
                    );
                }

                for (int i = 0; i < 4; i++) {
                    data.set(
                        index++,
                        false
                    );
                }
            }

            protected int getLength() {
                return 10;
            }

            protected BitSet getData() {
                return data;
            }
        }

        public static class SetPositionCommand extends Command {
            private final BitSet data;

            /**
             * @param bus the bus of the command's target device
             */
            public SetPositionCommand(int bus, float xPosition, float yPosition, float hOrientation) {
                data = new BitSet(getLength());

                data.set(
                    0,
                    (bus & 1) == 1
                );

                data.set(
                    1,
                    (bus >> 1 & 1) == 1
                );

                int index = 2;

                for (int i = 0; i < BITS_PER_I2C_DEVICE; i++) {
                    data.set(
                        index++,
                        (ID >> i & 1) == 1
                    );
                }

                for (int i = 0; i < 4; i++) {
                    data.set(
                        index++,
                        (1 >> i & 1) == 1
                    );
                }

                ByteBuffer buffer = ByteBuffer.allocate(12).order(BYTE_ORDER);

                buffer.putFloat(xPosition);
                buffer.putFloat(yPosition);
                buffer.putFloat(hOrientation);

                BitSet payload = BitSet.valueOf(buffer.array());

                for (int i = 0; i < 96; i++) {
                    data.set(
                        index++,
                        payload.get(i)
                    );
                }
            }

            protected int getLength() {
                return 106;
            }

            protected BitSet getData() {
                return data;
            }
        }

        private final BitSet config;

        public boolean disconnected = false;

        public short deviceStatus;

        public float xPosition;
        public float yPosition;
        public float hOrientation;

        public float xVelocity;
        public float yVelocity;
        public float hVelocity;

        /**
         * @param xPodOffset the offset of your forward tracking pod from the tracking center in millimeters
         * @param yPodOffset the offset of your strafe tracking pod from the tracking center in millimeters
         * @param encoderResolution the millimeters traveled per encoder tick
         * @param xEncoderDirection the direction of the forward encoder
         * @param yEncoderDirection the direction of the strafe encoder
         */
        public GoBildaPinpoint(
            float xPodOffset,
            float yPodOffset,
            float encoderResolution,
            EncoderDirection xEncoderDirection,
            EncoderDirection yEncoderDirection
        ) {
            ByteBuffer buffer = ByteBuffer.allocate(13).order(BYTE_ORDER);

            buffer.putFloat(xPodOffset);
            buffer.putFloat(yPodOffset);
            buffer.putFloat(encoderResolution);

            int directionBits = 0;

            if (xEncoderDirection == EncoderDirection.REVERSED) {
                directionBits |= 1;
            }

            if (yEncoderDirection == EncoderDirection.REVERSED) {
                directionBits |= 2;
            }

            buffer.put((byte) directionBits);

            config = BitSet.valueOf(buffer.array());
        }

        protected int getValue() {
            return ID;
        }

        protected int getInitLength() {
            return 98;
        }

        protected int getUpdateLength() {
            return 201;
        }

        protected int getAddress() {
            return 0x31;
        }

        protected BitSet getConfig() {
            return config;
        }

        protected void parseUpdate(BitSet data, int start) {
            int index = start;

            disconnected = data.get(index++);

            byte[] deviceStatusChunk = data
                .get(
                    index,
                    index + 8
                )
                .toByteArray();

            byte[] paddedDeviceStatusChunk = new byte[2];

            System.arraycopy(
                deviceStatusChunk,
                0,
                paddedDeviceStatusChunk,
                0,
                deviceStatusChunk.length
            );

            deviceStatus = ByteBuffer
                .wrap(paddedDeviceStatusChunk)
                .order(BYTE_ORDER)
                .getShort();

            index += 8;

            byte[] xPositionChunk = data
                .get(
                    index,
                    index + 32
                )
                .toByteArray();

            byte[] paddedXPositionChunk = new byte[4];

            System.arraycopy(
                xPositionChunk,
                0,
                paddedXPositionChunk,
                0,
                xPositionChunk.length
            );

            xPosition = ByteBuffer
                .wrap(paddedXPositionChunk)
                .order(BYTE_ORDER)
                .getFloat();

            index += 32;

            byte[] yPositionChunk = data
                .get(
                    index,
                    index + 32
                )
                .toByteArray();

            byte[] paddedYPositionChunk = new byte[4];

            System.arraycopy(
                yPositionChunk,
                0,
                paddedYPositionChunk,
                0,
                yPositionChunk.length
            );

            yPosition = ByteBuffer
                .wrap(paddedYPositionChunk)
                .order(BYTE_ORDER)
                .getFloat();

            index += 32;

            byte[] hOrientationChunk = data
                .get(
                    index,
                    index + 32
                )
                .toByteArray();

            byte[] paddedHOrientationChunk = new byte[4];

            System.arraycopy(
                hOrientationChunk,
                0,
                paddedHOrientationChunk,
                0,
                hOrientationChunk.length
            );

            hOrientation = ByteBuffer
                .wrap(paddedHOrientationChunk)
                .order(BYTE_ORDER)
                .getFloat();

            index += 32;

            byte[] xVelocityChunk = data
                .get(
                    index,
                    index + 32
                )
                .toByteArray();

            byte[] paddedXVelocityChunk = new byte[4];

            System.arraycopy(
                xVelocityChunk,
                0,
                paddedXVelocityChunk,
                0,
                xVelocityChunk.length
            );

            xVelocity = ByteBuffer
                .wrap(paddedXVelocityChunk)
                .order(BYTE_ORDER)
                .getFloat();

            index += 32;

            byte[] yVelocityChunk = data
                .get(
                    index,
                    index + 32
                )
                .toByteArray();

            byte[] paddedYVelocityChunk = new byte[4];

            System.arraycopy(
                yVelocityChunk,
                0,
                paddedYVelocityChunk,
                0,
                yVelocityChunk.length
            );

            yVelocity = ByteBuffer
                .wrap(paddedYVelocityChunk)
                .order(BYTE_ORDER)
                .getFloat();

            index += 32;

            byte[] hVelocityChunk = data
                .get(
                    index,
                    index + 32
                )
                .toByteArray();

            byte[] paddedHVelocityChunk = new byte[4];

            System.arraycopy(
                hVelocityChunk,
                0,
                paddedHVelocityChunk,
                0,
                hVelocityChunk.length
            );

            hVelocity = ByteBuffer
                .wrap(paddedHVelocityChunk)
                .order(BYTE_ORDER)
                .getFloat();
        }
    }

    public static class Config {
        private boolean locked = false;

        protected final AnalogDigitalDevice[] analogDigitalDevices =
            new AnalogDigitalDevice[12];

        protected final Encoder[] encoders = new Encoder[6];

        protected final ArrayList<I2CDevice>[] i2cBuses = new ArrayList[]{
            new ArrayList<I2CDevice>(),
            new ArrayList<I2CDevice>(),
            new ArrayList<I2CDevice>()
        };

        public Config() {
            Arrays.fill(
                analogDigitalDevices,
                AnalogDigitalDevice.NONE
            );

            Arrays.fill(
                encoders,
                Encoder.NONE
            );
        }

        /**
         * configures an analog-digital pin to be analog, digital, or none
         *
         * @param pin the pin being configured, from 1 to 12
         * @param device the type of device on the pin
         *
         * @throws IndexOutOfBoundsException if the pin is not between 1 and 12, inclusive
         * @throws IllegalStateException if init has already been called
         */
        public void setAnalogDigitalDevice(
            int pin,
            AnalogDigitalDevice device
        ) {
            if (pin < 1 || pin > 12) {
                throwException(
                    IndexOutOfBoundsException.class,
                    "AnalogDigitalDevice pin " +
                        "must be from 1 to 12"
                );
            }

            if (locked) {
                throwException(
                    IllegalStateException.class,
                    "Config has already been " +
                        "passed to the SRSHub; changes cannot be made"
                );
            }

            analogDigitalDevices[pin - 1] = device;
        }

        /**
         * configures an encoder port to be quadrature, pwm, or none
         *
         * @param port the port being configured, from 1 to 6
         * @param device the type of device on the port
         *
         * @throws IndexOutOfBoundsException if the port is not between 1 and 6, inclusive
         * @throws IllegalStateException if init has already been called
         */
        public void setEncoder(int port, Encoder device) {
            if (port < 1 || port > 6) {
                throwException(
                    IndexOutOfBoundsException.class,
                    "Encoder port must " +
                        "be from 1 to 6"
                );
            }

            if (locked) {
                throwException(
                    IllegalStateException.class,
                    "Config has already been " +
                        "passed to the SRSHub; changes cannot be made"
                );
            }

            encoders[port - 1] = device;
        }

        /**
         * adds a device to an I2C bus
         *
         * @param bus the bus to which the device is being added, from 1 to 3
         * @param device the (unique) type of the device on the bus
         *
         * @throws IndexOutOfBoundsException if the bus is not between 1 and 3, inclusive
         * @throws IllegalStateException if init has already been called or if a device of the same I2C address has been configured on the bus
         */
        public void addI2CDevice(int bus, I2CDevice device) {
            if (bus < 1 || bus > 3) {
                throwException(
                    IndexOutOfBoundsException.class,
                    "I2C bus must be from 1 to" +
                        " 3"
                );
            }

            if (locked) {
                throwException(
                    IllegalStateException.class,
                    "Config has already been " +
                        "passed to the SRSHub; changes cannot be made"
                );
            }

            for (I2CDevice i2cDevice : i2cBuses[bus - 1]) {
                if (i2cDevice.getClass() == device.getClass()) {
                    throwException(
                        IllegalStateException.class,
                        "I2C Bus #" + bus + " " +
                            "already has a device of type " + device
                            .getClass()
                            .getName()
                    );
                }

                if (i2cDevice.getAddress() == device.getAddress()) {
                    throwException(
                        IllegalStateException.class,
                        "I2C Bus #" + bus + " " +
                            "already has a bus of type " + i2cDevice
                            .getClass()
                            .getName() + " which has an I2C address conflicting " +
                            "with the " + device
                            .getClass()
                            .getName()
                    );
                }
            }

            i2cBuses[bus - 1].add(device);
        }

        protected void lock() {
            locked = true;
        }
    }

    public SRSHub(
        I2cDeviceSynchSimple deviceClient,
        boolean deviceClientIsOwned
    ) {
        super(
            deviceClient,
            deviceClientIsOwned
        );

        this.deviceClient.setI2cAddress(I2cAddr.create7bit(I2C_ADDRESS));
        super.registerArmingStateCallback(false);
    }

    protected boolean doInitialize() {
        ((LynxI2cDeviceSynch) this.deviceClient).setBusSpeed(LynxI2cDeviceSynch.BusSpeed.FAST_400K);

        isInitialized = false;

        verifyInitialization();

        return true;
    }

    public Manufacturer getManufacturer() {
        return Manufacturer.Other;
    }

    public String getDeviceName() {
        return "SRSHub";
    }

    enum Register {
        DEVICE_INFO(
            0x00,
            4
        ),

        RESTART(
            0x01,
            1
        ),

        INIT(
            0x02,
            -1
        ),

        READ(
            0x03,
            -1
        ),

        COMMAND(
            0x04,
            -1
        );

        public final byte address;
        public final int length;

        Register(int address, int length) {
            this.address = (byte) address;
            this.length = length;
        }
    }

    private void verifyInitialization() {
        if (!isInitialized) {
            byte[] deviceInfo = ByteBuffer.wrap(deviceClient.read(
                Register.DEVICE_INFO.address,
                Register.DEVICE_INFO.length
            )).order(BYTE_ORDER).array();

            int deviceId = deviceInfo[0];

            if (deviceId != DEVICE_ID) {
                RobotLog.addGlobalWarningMessage(
                    "SRSHub initialization failed"
                );

                disconnected = true;

                return;
            }

            int deviceMajorVersion = deviceInfo[1];
            int deviceMinorVersion = deviceInfo[2];
            int devicePatchVersion = deviceInfo[3];

            if (deviceMajorVersion != DEVICE_MAJOR_VERSION ||
                deviceMinorVersion != DEVICE_MINOR_VERSION ||
                devicePatchVersion != DEVICE_PATCH_VERSION) {
                throwException(
                    RuntimeException.class,
                    "SRSHub does not report correct firmware version; " +
                        "received v" + deviceMajorVersion + "."
                        + deviceMinorVersion + "." +
                        devicePatchVersion + ", expected v" +
                        DEVICE_MAJOR_VERSION + "." +
                        DEVICE_MINOR_VERSION + "." +
                        DEVICE_PATCH_VERSION
                );
            }

            isInitialized = true;
        }
    }

    private static void throwException(Class<? extends Exception> exception, String message) {
        RobotLog.setGlobalErrorMsg(message);

        try {
            throw (exception.getConstructor(String.class).newInstance(message));
        }
        catch (Exception e) {
            throw new RuntimeException(e);
        }
    }

    private int computeCRC16(byte[] data) {
        int crc = 0x0000;

        for (int i = 0; i < data.length - 2; i++) {
            crc ^= (data[i] & 0xFF) << 8;

            for (int j = 0; j < 8; j++) {
                if ((crc & 0x8000) != 0) {
                    crc = (crc << 1) ^ 0x1021;
                }
                else {
                    crc <<= 1;
                }

                crc &= 0xFFFF;
            }
        }

        return crc & 0xFFFF;
    }

    /**
     * passes the configuration to the SRSHub
     *
     * @param config the configuration details that will be passed to the SRSHub
     */
    public void init(
        @NonNull Config config
    ) {
        config.lock();
        this.config = config;

        ready = false;
        disconnected = false;

        updateLength = 8;

        deviceClient.write(
            Register.RESTART.address,
            new byte[Register.RESTART.length]
        );

        isInitialized = false;

        Arrays.fill(
            analogDigitalValues,
            0
        );

        for (int i = 0; i < encoderValues.length; i++) {
            encoderValues[i] = new PosVel();
        }

        int initLength =
            config.analogDigitalDevices.length * BITS_PER_ANALOG_DIGITAL_DEVICE + config.encoders.length * BITS_PER_ENCODER + config.i2cBuses.length * MIN_BITS_PER_I2C_BUS;

        int[] busLengths = new int[config.i2cBuses.length];

        for (int i = 0; i < config.i2cBuses.length; i++) {
            for (int j = 0; j < config.i2cBuses[i].size(); j++) {
                busLengths[i] += BITS_PER_I2C_DEVICE + config.i2cBuses[i]
                    .get(j)
                    .getInitLength();
            }

            initLength += busLengths[i];
        }

        BitSet init = new BitSet(initLength);

        int index = 0;

        for (int i = 0; i < config.analogDigitalDevices.length; i++) {
            switch (config.analogDigitalDevices[i]) {
                case ANALOG:
                    updateLength += 12;

                    break;
                case DIGITAL:
                    updateLength += 1;

                    break;
                case NONE:
                    break;
            }

            for (int j = 0; j < BITS_PER_ANALOG_DIGITAL_DEVICE; j++) {
                init.set(
                    index++,
                    (config.analogDigitalDevices[i].value >> j & 1) == 1
                );
            }
        }

        for (int i = 0; i < config.encoders.length; i++) {
            if (config.encoders[i] != Encoder.NONE) {
                updateLength += 48;
            }

            for (int j = 0; j < BITS_PER_ENCODER; j++) {
                init.set(
                    index++,
                    (config.encoders[i].value >> j & 1) == 1
                );
            }
        }

        for (int i = 0; i < config.i2cBuses.length; i++) {
            for (int j = 0; j < MIN_BITS_PER_I2C_BUS; j++) {
                init.set(
                    index++,
                    (busLengths[i] >> j & 1) == 1
                );
            }

            for (int j = 0; j < config.i2cBuses[i].size(); j++) {
                I2CDevice device = config.i2cBuses[i].get(j);

                updateLength += device.getUpdateLength();

                for (int k = 0; k < BITS_PER_I2C_DEVICE; k++) {
                    init.set(
                        index++,
                        (device.getValue() >> k & 1) == 1
                    );
                }

                for (int k = 0; k < device.getInitLength(); k++) {
                    init.set(
                        index++,
                        device.getConfig().get(k)
                    );
                }
            }
        }

        updateLength = 2 + (updateLength + 7) / 8;

        if (updateLength > 100) {
            throwException(
                IllegalStateException.class,
                "Maximum bulk-read length of 100 bytes exceeded"
            );
        }

        byte[] data = new byte[(initLength + 7) / 8];

        System.arraycopy(
            init.toByteArray(),
            0,
            data,
            0,
            init.toByteArray().length
        );

        try {
            sleep(2500);
        }
        catch (InterruptedException e) {
            throw new RuntimeException(e);
        }

        verifyInitialization();

        deviceClient.write(
            Register.INIT.address,
            data
        );

        update();

        ready = true;
    }

    /**
     * bulk-reads data from the SRSHub as specified in the configuration
     *
     * @throws IllegalStateException if the SRSHub has not yet been initialized
     * @throws RuntimeException if the SRSHub is unable to update according to the provided configuration
     */
    public void update() {
        if (config == null) {
            throwException(
                IllegalStateException.class,
                "The SRSHub must be initialized " +
                    "before updating"
            );
        }

        byte[] rawData = deviceClient.read(
            Register.READ.address,
            updateLength
        );

        if (rawData == null || rawData.length == 0 || rawData[0] != DEVICE_ID) {
            disconnected = true;

            return;
        }

        int receivedCRC = ((rawData[rawData.length - 2] & 0xFF) << 8) | (rawData[rawData.length - 1] & 0xFF);

        int computedCRC = computeCRC16(
            rawData
        );

        if (receivedCRC != computedCRC) {
            crcMismatchCount++;
            RobotLog.addGlobalWarningMessage("CRC Mismatch");
            return;
        }

        disconnected = false;

        BitSet data = BitSet.valueOf(rawData);

        int index = 8;

        for (int i = 0; i < config.analogDigitalDevices.length; i++) {
            switch (config.analogDigitalDevices[i]) {
                case ANALOG:
                    byte[] chunk = data
                        .get(
                            index,
                            index + 12
                        )
                        .toByteArray();

                    byte[] paddedChunk = new byte[2];

                    System.arraycopy(
                        chunk,
                        0,
                        paddedChunk,
                        0,
                        chunk.length
                    );

                    analogDigitalValues[i] = ByteBuffer
                        .wrap(paddedChunk)
                        .order(BYTE_ORDER)
                        .getShort() / (double) 4095;

                    index += 12;

                    break;
                case DIGITAL:
                    analogDigitalValues[i] = data.get(index++) ? 1 : 0;

                    break;
                case NONE:
                    break;
            }
        }

        for (int i = 0; i < config.encoders.length; i++) {
            switch (config.encoders[i]) {
                case QUADRATURE:
                    int lastPosition = encoderValues[i].position;

                    encoderValues[i] = new PosVel();

                    byte[] quadratureChunk = data
                        .get(
                            index,
                            index + 16
                        ).toByteArray();

                    byte[] paddedQuadratureChunk = new byte[2];

                    System.arraycopy(
                        quadratureChunk,
                        0,
                        paddedQuadratureChunk,
                        0,
                        quadratureChunk.length
                    );

                    encoderValues[i].position = lastPosition + ByteBuffer
                        .wrap(paddedQuadratureChunk)
                        .order(BYTE_ORDER)
                        .getShort();

                    index += 16;

                    byte[] quadratureVelocityChunk = data
                        .get(
                            index,
                            index + 32
                        ).toByteArray();

                    byte[] paddedQuadratureVelocityChunk = new byte[4];

                    System.arraycopy(
                        quadratureVelocityChunk,
                        0,
                        paddedQuadratureVelocityChunk,
                        0,
                        quadratureVelocityChunk.length
                    );

                    encoderValues[i].velocity = ByteBuffer
                        .wrap(paddedQuadratureVelocityChunk)
                        .order(BYTE_ORDER)
                        .getInt();

                    index += 32;

                    break;
                case PWM:
                    encoderValues[i] = new PosVel();

                    byte[] pwmChunk = data
                        .get(
                            index,
                            index + 16
                        ).toByteArray();

                    byte[] paddedPWMChunk = new byte[2];

                    System.arraycopy(
                        pwmChunk,
                        0,
                        paddedPWMChunk,
                        0,
                        pwmChunk.length
                    );

                    encoderValues[i].position = ByteBuffer
                        .wrap(paddedPWMChunk)
                        .order(BYTE_ORDER)
                        .getShort();

                    index += 16;

                    byte[] pwmVelocityChunk = data
                        .get(
                            index,
                            index + 32
                        ).toByteArray();

                    byte[] paddedPWMVelocityChunk = new byte[4];

                    System.arraycopy(
                        pwmVelocityChunk,
                        0,
                        paddedPWMVelocityChunk,
                        0,
                        pwmVelocityChunk.length
                    );

                    encoderValues[i].velocity = ByteBuffer
                        .wrap(paddedPWMVelocityChunk)
                        .order(BYTE_ORDER)
                        .getInt();

                    index += 32;

                    break;
                case NONE:
                    break;
            }
        }

        for (int i = 0; i < config.i2cBuses.length; i++) {
            for (int j = 0; j < config.i2cBuses[i].size(); j++) {
                I2CDevice device = config.i2cBuses[i]
                    .get(j);

                device.parseUpdate(
                    data,
                    index
                );
                index += device.getUpdateLength();
            }
        }
    }

    /**
     * @return whether the SRSHub is done initializing
     */
    public boolean ready() {
        return ready;
    }

    /**
     * @return whether the most recent update failed
     */
    public boolean disconnected() {
        return disconnected;
    }

    /**
     * @return how many updates have been dropped due to a CRC mismatch
     */
    public long crcMismatchCount() {
        return crcMismatchCount;
    }

    /**
     * gets the current value of the AnalogDigitalDevice at the specified pin
     *
     * @param pin the pin being read, from 1 to 12
     *
     * @return the current value read from the AnalogDigitalDevice; from 0 to 1 for analog devices and 0 or 1 for digital devices
     *
     * @throws IndexOutOfBoundsException if the pin is not between 1 and 12, inclusive
     * @throws IllegalStateException if the SRSHub has not yet been initialized
     * @throws IllegalStateException if the pin was not configured
     */
    public double readAnalogDigitalDevice(int pin) {
        if (pin < 1 || pin > 12) {
            throwException(
                IndexOutOfBoundsException.class,
                "AnalogDigitalDevice pin " +
                    "must be from 1 to 12"
            );
        }

        if (config == null) {
            throwException(
                IllegalStateException.class,
                "The SRSHub must be initialized " +
                    "before reading"
            );
        }

        if (config.analogDigitalDevices[pin - 1] == AnalogDigitalDevice.NONE) {
            throwException(
                IllegalStateException.class,
                "AnalogDigitalDevice pin #" + pin +
                    " was not configured"
            );
        }

        return analogDigitalValues[pin - 1];
    }

    /**
     * gets the current position and velocity of the encoder at the specified port
     *
     * @param port the port being read, from 1 to 6
     *
     * @return the current position and velocity of the encoder; for quadrature encoders this is in ticks/ticks per second; for PWM encoders this is in pulse
     *     width (microseconds/microseconds per second)
     *
     * @throws IndexOutOfBoundsException if the port is not between 1 and 6, inclusive
     * @throws IllegalStateException if the SRSHub has not yet been initialized
     * @throws IllegalStateException if the port was not configured
     */
    public PosVel readEncoder(int port) {
        if (port < 1 || port > 6) {
            throwException(
                IndexOutOfBoundsException.class,
                "Encoder port " +
                    "must be from 1 to 6"
            );
        }

        if (config == null) {
            throwException(
                IllegalStateException.class,
                "The SRSHub must be initialized " +
                    "before reading"
            );
        }

        if (config.encoders[port - 1] == Encoder.NONE) {
            throwException(
                IllegalStateException.class,
                "Encoder port #" + port +
                    " was not configured"
            );
        }

        return encoderValues[port - 1];
    }

    /**
     * gets the current value(s) read from the specified I2C device at the specified bus
     *
     * @param bus the bus from which the device is being read, from 1 to 3
     * @param deviceClass the type of device being read
     *
     * @return a wrapper for the current value(s) returned by the I2C device
     *
     * @throws IndexOutOfBoundsException if the bus is not between 1 and 3, inclusive
     * @throws IllegalStateException if the SRSHub has not yet been initialized
     * @throws IllegalStateException if the device was not configured on the bus
     */
    public <T extends I2CDevice> T getI2CDevice(int bus, Class<T> deviceClass) {
        if (bus < 1 || bus > 3) {
            throwException(
                IndexOutOfBoundsException.class,
                "I2C bus must be from 1 to 3"
            );
        }

        if (config == null) {
            throwException(
                IllegalStateException.class,
                "The SRSHub must be initialized before reading"
            );
        }

        for (I2CDevice device : config.i2cBuses[bus - 1]) {
            if (deviceClass.isInstance(device)) {
                return deviceClass.cast(device);
            }
        }

        throwException(
            IllegalStateException.class,
            "I2C device " + deviceClass.getName() +
                " was not configured on bus #" + bus
        );

        return null;
    }

    /**
     * runs a command on an I2C device
     *
     * @throws IllegalStateException if the SRSHub has not yet been initialized
     */
    public void runCommand(Command command) {
        if (config == null) {
            throwException(
                IllegalStateException.class,
                "The SRSHub must be initialized before reading"
            );
        }

        byte[] data = new byte[(command.getLength() + 7) / 8];

        System.arraycopy(
            command.getData().toByteArray(),
            0,
            data,
            0,
            command.getData().toByteArray().length
        );

        deviceClient.write(
            Register.COMMAND.address,
            data
        );
    }
}
