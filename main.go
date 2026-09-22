package main

import (
	"flag"
	"fmt"
	"time"

	"github.com/karalabe/hid"
	"go.bug.st/serial"

	"tx12-bandit/logger"
)

const (
	VID = 0x1209
	PID = 0x4F54

	ReportSize = 19

	AxisMin = 0
	AxisMax = 2047

	CRSFMin    = 172
	CRSFCenter = 992
	CRSFMax    = 1811

	CRSFAddress = 0xC8
	CRSFTypeRC  = 0x16

	CRSFPayloadSize = 22
	CRSFFrameSize   = 26

	UARTPort     = "/dev/ttyUSB0"
	UARTBaudRate = 420000

	CRSFInterval = 5 * time.Millisecond

	LoopbackReadTimeout = 20 * time.Millisecond
)

// ============================================================
// HID REPORT
// ============================================================

type HIDReport struct {
	Buttons uint32

	X  uint16
	Y  uint16
	Z  uint16
	RX uint16
	RY uint16
	RZ uint16

	Slider1 uint16
	Slider2 uint16
}

// ============================================================
// HID
// ============================================================

func u16LE(buf []byte, offset int) uint16 {
	return uint16(buf[offset]) |
		(uint16(buf[offset+1]) << 8)
}

func clampAxis(v uint16) uint16 {
	if v > AxisMax {
		return AxisMax
	}

	return v
}

func parseReport(buf []byte) HIDReport {
	var r HIDReport

	if len(buf) < ReportSize {
		return r
	}

	// 24 buttons = 3 bytes.
	r.Buttons =
		uint32(buf[0]) |
			uint32(buf[1])<<8 |
			uint32(buf[2])<<16

	// 8 × 16-bit axes.
	r.X = clampAxis(u16LE(buf, 3))
	r.Y = clampAxis(u16LE(buf, 5))
	r.Z = clampAxis(u16LE(buf, 7))
	r.RX = clampAxis(u16LE(buf, 9))
	r.RY = clampAxis(u16LE(buf, 11))
	r.RZ = clampAxis(u16LE(buf, 13))

	r.Slider1 = clampAxis(u16LE(buf, 15))
	r.Slider2 = clampAxis(u16LE(buf, 17))

	return r
}

// ============================================================
// HID -> CRSF
// ============================================================

func hidToCRSF(v uint16) uint16 {
	if v <= 1024 {
		return uint16(
			CRSFMin+
				int(v)*int(CRSFCenter-CRSFMin)/1024,
		)
	}

	return uint16(
		CRSFCenter+
			int(v-1024)*int(CRSFMax-CRSFCenter)/(2047-1024),
	)
}

// ============================================================
// CRC-8/D5
// ============================================================

func crc8(data []byte) byte {
	var crc byte

	for _, b := range data {
		crc ^= b

		for i := 0; i < 8; i++ {
			if crc&0x80 != 0 {
				crc = (crc << 1) ^ 0xD5
			} else {
				crc <<= 1
			}
		}
	}

	return crc
}

// ============================================================
// CRSF PACK
// ============================================================

func packChannels(channels [16]uint16) [CRSFPayloadSize]byte {
	var payload [CRSFPayloadSize]byte

	bitPos := 0

	for _, channel := range channels {
		value := channel & 0x07FF

		for bit := 0; bit < 11; bit++ {
			if value&(1<<bit) != 0 {
				bytePos := bitPos / 8
				bitOffset := bitPos % 8

				payload[bytePos] |= 1 << bitOffset
			}

			bitPos++
		}
	}

	return payload
}

// ============================================================
// CRSF UNPACK
// ============================================================

func unpackChannels(payload []byte) [16]uint16 {
	var channels [16]uint16

	if len(payload) < CRSFPayloadSize {
		return channels
	}

	bitPos := 0

	for channel := 0; channel < 16; channel++ {
		var value uint16

		for bit := 0; bit < 11; bit++ {
			bytePos := bitPos / 8
			bitOffset := bitPos % 8

			if payload[bytePos]&(1<<bitOffset) != 0 {
				value |= 1 << bit
			}

			bitPos++
		}

		channels[channel] = value
	}

	return channels
}

// ============================================================
// CREATE CRSF FRAME
// ============================================================

func makeCRSF(channels [16]uint16) []byte {
	payload := packChannels(channels)

	frame := make([]byte, CRSFFrameSize)

	frame[0] = CRSFAddress
	frame[1] = 0x18
	frame[2] = CRSFTypeRC

	copy(frame[3:25], payload[:])

	// CRC = TYPE + PAYLOAD.
	frame[25] = crc8(frame[2:25])

	return frame
}

// ============================================================
// VALIDATE CRSF
// ============================================================

func validateCRSF(frame []byte) error {
	if len(frame) != CRSFFrameSize {
		return fmt.Errorf(
			"неверный размер frame: %d, ожидается %d",
			len(frame),
			CRSFFrameSize,
		)
	}

	if frame[0] != CRSFAddress {
		return fmt.Errorf(
			"неверный ADDRESS: 0x%02X, ожидается 0x%02X",
			frame[0],
			CRSFAddress,
		)
	}

	if frame[1] != 0x18 {
		return fmt.Errorf(
			"неверный LENGTH: 0x%02X, ожидается 0x18",
			frame[1],
		)
	}

	if frame[2] != CRSFTypeRC {
		return fmt.Errorf(
			"неверный TYPE: 0x%02X, ожидается 0x16",
			frame[2],
		)
	}

	expectedCRC := crc8(frame[2:25])
	actualCRC := frame[25]

	if expectedCRC != actualCRC {
		return fmt.Errorf(
			"неверный CRC: получен 0x%02X, ожидается 0x%02X",
			actualCRC,
			expectedCRC,
		)
	}

	return nil
}

// ============================================================
// VALIDATE PACKING
// ============================================================

func validatePacking(channels [16]uint16, frame []byte) error {
	if len(frame) != CRSFFrameSize {
		return fmt.Errorf("неверный размер frame")
	}

	decoded := unpackChannels(frame[3:25])

	for i := 0; i < 16; i++ {
		expected := channels[i] & 0x07FF
		actual := decoded[i]

		if expected != actual {
			return fmt.Errorf(
				"ошибка CH%d: исходное=%d, распакованное=%d",
				i+1,
				expected,
				actual,
			)
		}
	}

	return nil
}

// ============================================================
// HID -> CHANNELS
// ============================================================

func makeChannels(report HIDReport) [16]uint16 {
	var channels [16]uint16

	// Left stick.
	leftX := report.RX
	leftY := report.Z

	// Right stick.
	rightX := report.X
	rightY := report.Y

	// CH1..CH4.
	channels[0] = hidToCRSF(leftX)
	channels[1] = hidToCRSF(leftY)
	channels[2] = hidToCRSF(rightX)
	channels[3] = hidToCRSF(rightY)

	// CH5..CH16 = center.
	for i := 4; i < 16; i++ {
		channels[i] = CRSFCenter
	}

	return channels
}

// ============================================================
// COMPARE FRAMES
// ============================================================

func compareFrames(expected []byte, actual []byte) error {
	if len(expected) != len(actual) {
		return fmt.Errorf(
			"разный размер: TX=%d RX=%d",
			len(expected),
			len(actual),
		)
	}

	for i := 0; i < len(expected); i++ {
		if expected[i] != actual[i] {
			return fmt.Errorf(
				"различие на byte[%d]: TX=0x%02X RX=0x%02X",
				i,
				expected[i],
				actual[i],
			)
		}
	}

	return nil
}

// ============================================================
// READ CRSF FRAME FROM UART STREAM
//
// UART является потоком байтов.
// Read() НЕ обязан начинаться с начала CRSF frame.
//
// Поэтому ищем:
//
//     C8 18 16
//
// Затем проверяем полный 26-byte frame и CRC.
//
// Это исправляет ситуацию:
//
//     RX: 07 3E F0 ... AD C8 18 16 ...
//
// когда физический loopback работает, но Read()
// начал чтение не с первого байта пакета.
// ============================================================

func readCRSFFrame(
	uart serial.Port,
	timeout time.Duration,
) ([]byte, error) {

	if err := uart.SetReadTimeout(2 * time.Millisecond); err != nil {
		return nil, fmt.Errorf(
			"не удалось установить UART read timeout: %w",
			err,
		)
	}

	deadline := time.Now().Add(timeout)

	stream := make([]byte, 0, CRSFFrameSize*2)
	tmp := make([]byte, 64)

	for time.Now().Before(deadline) {

		n, err := uart.Read(tmp)

		if err != nil {
			return nil, fmt.Errorf(
				"ошибка UART Read: %w",
				err,
			)
		}

		if n == 0 {
			continue
		}

		stream = append(stream, tmp[:n]...)

		// ----------------------------------------------------
		// Ищем полный 26-byte CRSF frame.
		// ----------------------------------------------------

		for i := 0; i+CRSFFrameSize <= len(stream); i++ {

			// ADDRESS
			if stream[i] != CRSFAddress {
				continue
			}

			// LENGTH
			if stream[i+1] != 0x18 {
				continue
			}

			// TYPE
			if stream[i+2] != CRSFTypeRC {
				continue
			}

			// Полный frame.
			candidate := stream[i : i+CRSFFrameSize]

			// Проверяем CRC.
			if err := validateCRSF(candidate); err != nil {
				continue
			}

			// Копируем найденный frame.
			frame := make([]byte, CRSFFrameSize)

			copy(frame, candidate)

			return frame, nil
		}

		// ----------------------------------------------------
		// Сохраняем последние 25 байт.
		//
		// Это нужно, если начало frame оказалось
		// разделено между двумя Read().
		// ----------------------------------------------------

		if len(stream) > CRSFFrameSize-1 {

			stream = append(
				[]byte(nil),
				stream[len(stream)-(CRSFFrameSize-1):]...,
			)
		}
	}

	return nil, fmt.Errorf(
		"timeout: CRSF frame не найден за %s",
		timeout,
	)
}

// ============================================================
// PHYSICAL UART LOOPBACK TEST
//
// TX -> physical wire -> RX
//
// TX12 НЕ ИСПОЛЬЗУЕТСЯ.
// ============================================================

func runUARTLoopback(uart serial.Port) {

	logger.Logger.Header("CRSF UART LOOPBACK TEST")

	logger.Logger.Info("TX12      : DISABLED")
	logger.Logger.Info("UART      : %s", UARTPort)
	logger.Logger.Info("BAUD      : %d", UARTBaudRate)
	logger.Logger.Info("FORMAT    : 8N1")
	logger.Logger.Info("RATE      : 200 Hz")
	logger.Logger.Info("INTERVAL  : %s", CRSFInterval)

	var channels [16]uint16

	for i := 0; i < 16; i++ {
		channels[i] = CRSFCenter
	}

	logger.Logger.Info("TEST CHANNELS:")
	logger.Logger.Channels(channels)

	// --------------------------------------------------------
	// Создаём frame.
	// --------------------------------------------------------

	frame := makeCRSF(channels)

	// --------------------------------------------------------
	// Проверяем TX frame.
	// --------------------------------------------------------

	if err := validateCRSF(frame); err != nil {
		logger.Logger.Fatal(
			"TX FRAME CHECK FAIL: %v",
			err,
		)
	}

	if err := validatePacking(channels, frame); err != nil {
		logger.Logger.Fatal(
			"TX PACK CHECK FAIL: %v",
			err,
		)
	}

	logger.Logger.Info("TX FRAME:")
	logger.Logger.CRSF("TX", frame)

	logger.Logger.Info(
		"TX SIZE : %d bytes",
		len(frame),
	)

	logger.Logger.Info(
		"TX CRC  : 0x%02X",
		frame[25],
	)

	// --------------------------------------------------------
	// Очищаем RX/TX buffers.
	// --------------------------------------------------------

	if err := uart.ResetInputBuffer(); err != nil {
		logger.Logger.Fatal(
			"не удалось очистить UART RX buffer: %v",
			err,
		)
	}

	if err := uart.ResetOutputBuffer(); err != nil {
		logger.Logger.Fatal(
			"не удалось очистить UART TX buffer: %v",
			err,
		)
	}

	// --------------------------------------------------------
	// PHYSICAL LOOPBACK CHECK
	// --------------------------------------------------------

	logger.Logger.Separator()
	logger.Logger.Info("PHYSICAL LOOPBACK CHECK")
	logger.Logger.Separator()

	logger.Logger.Info(
		"Отправляем %d байт через TX...",
		CRSFFrameSize,
	)

	n, err := uart.Write(frame)

	if err != nil {
		logger.Logger.Fatal(
			"ошибка записи UART: %v",
			err,
		)
	}

	if n != CRSFFrameSize {
		logger.Logger.Fatal(
			"UART записал %d байт, ожидалось %d",
			n,
			CRSFFrameSize,
		)
	}

	logger.Logger.Info(
		"TX SENT : %d bytes",
		n,
	)

	// --------------------------------------------------------
	// Читаем UART как поток.
	//
	// НЕ используем readExact().
	// --------------------------------------------------------

	rxFrame, err := readCRSFFrame(
		uart,
		LoopbackReadTimeout,
	)

	if err != nil {

		logger.Logger.Error(
			"RX ERROR: %v",
			err,
		)

		logger.Logger.Error(
			"Проверь физическое соединение:",
		)

		logger.Logger.Error(
			"  USB-UART TX  --->  RX",
		)

		logger.Logger.Error(
			"  USB-UART RX  <---  TX",
		)

		logger.Logger.Fatal(
			"физический UART loopback не получен",
		)
	}

	logger.Logger.Info(
		"RX RECV : %d bytes",
		len(rxFrame),
	)

	logger.Logger.CRSF(
		"RX",
		rxFrame,
	)

	// --------------------------------------------------------
	// Проверяем RX CRSF.
	// --------------------------------------------------------

	if err := validateCRSF(rxFrame); err != nil {

		logger.Logger.Fatal(
			"RX FRAME CHECK FAIL: %v",
			err,
		)
	}

	logger.Logger.Info(
		"RX FRAME CHECK : OK",
	)

	// --------------------------------------------------------
	// Сравниваем TX и RX.
	// --------------------------------------------------------

	if err := compareFrames(
		frame,
		rxFrame,
	); err != nil {

		logger.Logger.Error(
			"FRAME MATCH : NO",
		)

		logger.Logger.Error(
			"ERROR       : %v",
			err,
		)

		logger.Logger.Info("TX:")
		logger.Logger.CRSF("TX", frame)

		logger.Logger.Info("RX:")
		logger.Logger.CRSF("RX", rxFrame)

		logger.Logger.Fatal(
			"UART LOOPBACK FRAME DIFFERENT",
		)
	}

	logger.Logger.Info(
		"FRAME MATCH : YES",
	)

	// --------------------------------------------------------
	// Проверяем packing.
	// --------------------------------------------------------

	if err := validatePacking(
		channels,
		rxFrame,
	); err != nil {

		logger.Logger.Fatal(
			"RX PACK CHECK FAIL: %v",
			err,
		)
	}

	logger.Logger.Info(
		"RX PACK CHECK : OK",
	)

	logger.Logger.Info(
		"RX CRC : OK",
	)

	logger.Logger.Info(
		"RX CRC : 0x%02X",
		rxFrame[25],
	)

	// --------------------------------------------------------
	// LOOPBACK TEST PASSED.
	// --------------------------------------------------------

	logger.Logger.Header(
		"UART LOOPBACK TEST PASSED",
	)

	logger.Logger.Info(
		"26/26 bytes identical.",
	)

	logger.Logger.Info(
		"TX == RX",
	)

	logger.Logger.Info(
		"CRSF synchronization : OK",
	)

	logger.Logger.Info(
		"CRC valid.",
	)

	logger.Logger.Info(
		"Physical TX -> RX loopback : OK",
	)

	// --------------------------------------------------------
	// CONTINUOUS TRANSMISSION
	//
	// Отправляем один и тот же frame каждые 5 ms.
	// TX12 не используется.
	// --------------------------------------------------------

	logger.Logger.Info(
		"Начинаем непрерывную передачу...",
	)

	logger.Logger.Info(
		"Отправляется один и тот же CRSF frame:",
	)

	logger.Logger.CRSF(
		"CRSF",
		frame,
	)

	ticker := time.NewTicker(
		CRSFInterval,
	)

	defer ticker.Stop()

	var frameCounter uint64

	for range ticker.C {

		n, err := uart.Write(frame)

		if err != nil {
			logger.Logger.Fatal(
				"ошибка записи UART: %v",
				err,
			)
		}

		if n != CRSFFrameSize {
			logger.Logger.Fatal(
				"UART записал %d байт, ожидалось %d",
				n,
				CRSFFrameSize,
			)
		}

		frameCounter++

		if frameCounter%20 == 0 {

			logger.Logger.Info(
				"LOOPBACK TX: frame #%d | %d bytes | CRC=0x%02X",
				frameCounter,
				n,
				frame[25],
			)
		}
	}
}

// ============================================================
// NORMAL HID MODE
// ============================================================

func runHIDMode(uart serial.Port) {

	// --------------------------------------------------------
	// Открываем TX12 HID.
	// --------------------------------------------------------

	devices := hid.Enumerate(VID, PID)

	if len(devices) == 0 {
		logger.Logger.Fatal(
			"TX12 не найден",
		)
	}

	logger.Logger.Info(
		"Найдено HID устройств: %d",
		len(devices),
	)

	for i, device := range devices {

		logger.Logger.Info(
			"[%d] %s - %s",
			i,
			device.Manufacturer,
			device.Product,
		)
	}

	d, err := devices[0].Open()

	if err != nil {
		logger.Logger.Fatal(
			"не удалось открыть TX12: %v",
			err,
		)
	}

	defer d.Close()

	logger.Logger.Info(
		"TX12 подключен",
	)

	// --------------------------------------------------------
	// Configuration.
	// --------------------------------------------------------

	logger.Logger.Info("UART:")

	logger.Logger.Info(
		"  Port       : %s",
		UARTPort,
	)

	logger.Logger.Info(
		"  Baud       : %d",
		UARTBaudRate,
	)

	logger.Logger.Info(
		"  Format     : 8N1",
	)

	logger.Logger.Info("CRSF bridge:")

	logger.Logger.Info(
		"  CH1 = Left X  = RX",
	)

	logger.Logger.Info(
		"  CH2 = Left Y  = Z",
	)

	logger.Logger.Info(
		"  CH3 = Right X = X",
	)

	logger.Logger.Info(
		"  CH4 = Right Y = Y",
	)

	logger.Logger.Info(
		"  CH5..CH16 = 992",
	)

	logger.Logger.Info("CRSF:")

	logger.Logger.Info(
		"  Frame size : %d",
		CRSFFrameSize,
	)

	logger.Logger.Info(
		"  Payload    : %d",
		CRSFPayloadSize,
	)

	logger.Logger.Info(
		"  Channels   : 16 × 11 bit",
	)

	logger.Logger.Info(
		"  Type       : 0x16",
	)

	logger.Logger.Info(
		"  Address    : 0xC8",
	)

	logger.Logger.Info(
		"  CRC        : CRC-8/D5",
	)

	logger.Logger.Info(
		"  Rate       : 200 Hz",
	)

	logger.Logger.Info(
		"Запуск...",
	)

	// --------------------------------------------------------
	// HID buffer.
	// --------------------------------------------------------

	buf := make([]byte, 64)

	// --------------------------------------------------------
	// HID reports.
	// --------------------------------------------------------

	hidReports := make(
		chan HIDReport,
		1,
	)

	hidErrors := make(
		chan error,
		1,
	)

	go func() {

		for {

			n, err := d.Read(buf)

			if err != nil {

				hidErrors <- err

				return
			}

			if n < ReportSize {
				continue
			}

			reportData := make(
				[]byte,
				n,
			)

			copy(
				reportData,
				buf[:n],
			)

			report := parseReport(
				reportData,
			)

			select {

			case hidReports <- report:

			default:

				select {
				case <-hidReports:
				default:
				}

				hidReports <- report
			}
		}
	}()

	// --------------------------------------------------------
	// Initial channels.
	// --------------------------------------------------------

	var channels [16]uint16

	for i := 0; i < 16; i++ {
		channels[i] = CRSFCenter
	}

	// --------------------------------------------------------
	// UART ticker = 200 Hz.
	// --------------------------------------------------------

	ticker := time.NewTicker(
		CRSFInterval,
	)

	defer ticker.Stop()

	var frameCounter uint64

	for {

		select {

		// ----------------------------------------------------
		// Новые данные TX12.
		// ----------------------------------------------------

		case report := <-hidReports:

			channels = makeChannels(
				report,
			)

		// ----------------------------------------------------
		// Ошибка HID.
		// ----------------------------------------------------

		case err := <-hidErrors:

			logger.Logger.Fatal(
				"ошибка HID: %v",
				err,
			)

		// ----------------------------------------------------
		// Каждые 5 ms отправляем CRSF.
		// ----------------------------------------------------

		case <-ticker.C:

			frame := makeCRSF(
				channels,
			)

			if err := validateCRSF(frame); err != nil {

				logger.Logger.Fatal(
					"FRAME CHECK FAIL: %v",
					err,
				)
			}

			if err := validatePacking(
				channels,
				frame,
			); err != nil {

				logger.Logger.Fatal(
					"PACK CHECK FAIL: %v",
					err,
				)
			}

			// ------------------------------------------------
			// Отправляем ровно 26 байт в Bandit.
			// ------------------------------------------------

			n, err := uart.Write(frame)

			if err != nil {

				logger.Logger.Fatal(
					"ошибка записи UART: %v",
					err,
				)
			}

			if n != CRSFFrameSize {

				logger.Logger.Fatal(
					"UART записал %d байт, ожидалось %d",
					n,
					CRSFFrameSize,
				)
			}

			frameCounter++

			// ------------------------------------------------
			// Диагностика примерно 10 раз в секунду.
			// ------------------------------------------------

			if frameCounter%20 == 0 {

				logger.Logger.Info(
					"LEFT CRSF : X=%4d Y=%4d | RIGHT CRSF: X=%4d Y=%4d",
					channels[0],
					channels[1],
					channels[2],
					channels[3],
				)

				logger.Logger.Channels(
					channels,
				)

				logger.Logger.Payload(
					frame,
				)

				logger.Logger.CRSF(
					"CRSF HEX",
					frame,
				)

				logger.Logger.Info(
					"UART      : %d bytes sent | CRC=0x%02X | frame #%d",
					n,
					frame[25],
					frameCounter,
				)

				logger.Logger.Separator()
			}
		}
	}
}

// ============================================================
// MAIN
// ============================================================

func main() {

	// --------------------------------------------------------
	// FLAGS
	// --------------------------------------------------------

	loopback := flag.Bool(
		"loopback",
		false,
		"физический UART loopback без TX12: TX -> RX и проверка 26 байт",
	)

	flag.Parse()

	// --------------------------------------------------------
	// UART CONFIGURATION
	// --------------------------------------------------------

	mode := &serial.Mode{
		BaudRate: UARTBaudRate,
		DataBits: 8,
		Parity:   serial.NoParity,
		StopBits: serial.OneStopBit,
	}

	// --------------------------------------------------------
	// OPEN UART
	// --------------------------------------------------------

	uart, err := serial.Open(
		UARTPort,
		mode,
	)

	if err != nil {

		logger.Logger.Fatal(
			"не удалось открыть UART %s: %v",
			UARTPort,
			err,
		)
	}

	defer uart.Close()

	// --------------------------------------------------------
	// LOOPBACK MODE
	//
	// TX12 здесь вообще НЕ открывается.
	// --------------------------------------------------------

	if *loopback {

		runUARTLoopback(uart)

		return
	}

	// --------------------------------------------------------
	// NORMAL HID MODE
	// --------------------------------------------------------

	runHIDMode(uart)
}