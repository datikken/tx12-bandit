package main

import (
	"flag"
	"fmt"
	"io"
	"log"
	"time"

	"tx12-bandit/udpoutput"

	"github.com/karalabe/hid"
	"go.bug.st/serial"
)

const (
	VID = 0x1209
	PID = 0x4F54

	ReportSize = 19

	// HID axis range.
	AxisMin = 0
	AxisMax = 2047

	// CRSF RC channel range.
	CRSFMin    = 172
	CRSFCenter = 992
	CRSFMax    = 1811

	// CRSF frame.
	CRSFAddress = 0xC8
	CRSFTypeRC  = 0x16

	// 16 channels × 11 bits = 176 bits = 22 bytes.
	CRSFPayloadSize = 22
	CRSFFrameSize   = 26

	// UART.
	UARTPort = "/dev/ttyUSB0"

	// Standard CRSF UART speed.
	UARTBaudRate = 420000

	// Send CRSF at 200 Hz = every 5 ms.
	CRSFInterval = 5 * time.Millisecond
)

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

// ------------------------------------------------------------
// HID 0..2047 -> CRSF 172..1811
//
// HID:
//   0    -> 172
//   1024 -> 992
//   2047 -> 1811
// ------------------------------------------------------------

func hidToCRSF(v uint16) uint16 {
	if v <= 1024 {
		return uint16(
			CRSFMin +
				int(v)*int(CRSFCenter-CRSFMin)/1024,
		)
	}

	return uint16(
		CRSFCenter +
			int(v-1024)*int(CRSFMax-CRSFCenter)/(2047-1024),
	)
}

// ------------------------------------------------------------
// CRSF CRC-8/D5
// ------------------------------------------------------------

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

// ------------------------------------------------------------
// Упаковка 16 × 11 bit.
// ------------------------------------------------------------

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

// ------------------------------------------------------------
// Распаковка 22 байт обратно в 16 × 11 bit.
// ------------------------------------------------------------

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

// ------------------------------------------------------------
// Создание полного CRSF frame.
//
// 0     ADDRESS   C8
// 1     LENGTH    18
// 2     TYPE      16
// 3..24 PAYLOAD   22 bytes
// 25    CRC
//
// Всего = 26 байт.
// ------------------------------------------------------------

func makeCRSF(channels [16]uint16) []byte {
	payload := packChannels(channels)

	frame := make([]byte, CRSFFrameSize)

	frame[0] = CRSFAddress
	frame[1] = 0x18
	frame[2] = CRSFTypeRC

	copy(frame[3:25], payload[:])

	// CRC считается от TYPE + PAYLOAD.
	frame[25] = crc8(frame[2:25])

	return frame
}

func sendUART(uart io.Writer, frame []byte) (int, error) {
	n, err := uart.Write(frame)
	if err != nil {
		return n, fmt.Errorf("ошибка записи UART: %w", err)
	}

	if n != CRSFFrameSize {
		return n, fmt.Errorf(
			"UART записал %d байт, ожидалось %d",
			n,
			CRSFFrameSize,
		)
	}

	return n, nil
}

func sendUDP(udpSender *udpoutput.Sender, frame []byte) error {
	if udpSender == nil {
		return nil
	}

	if err := udpSender.Send(frame); err != nil {
		return fmt.Errorf("ошибка записи UDP: %w", err)
	}

	return nil
}

// ------------------------------------------------------------
// Проверка полного CRSF frame.
// ------------------------------------------------------------

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

// ------------------------------------------------------------
// Проверка упаковки.
// ------------------------------------------------------------

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

// ------------------------------------------------------------
// Вывод каналов.
// ------------------------------------------------------------

func printChannels(channels [16]uint16) {
	fmt.Print("CRSF channels:")

	for i, channel := range channels {
		fmt.Printf(
			" CH%d=%4d",
			i+1,
			channel,
		)
	}

	fmt.Println()
}

// ------------------------------------------------------------
// Вывод HEX.
// ------------------------------------------------------------

func printCRSF(frame []byte) {
	fmt.Print("CRSF HEX: ")

	for i, b := range frame {
		fmt.Printf("%02X", b)

		if i < len(frame)-1 {
			fmt.Print(" ")
		}
	}

	fmt.Println()
}

// ------------------------------------------------------------
// Вывод payload.
// ------------------------------------------------------------

func printPayload(frame []byte) {
	fmt.Print("PAYLOAD : ")

	for i := 3; i < 25; i++ {
		fmt.Printf("%02X", frame[i])

		if i < 24 {
			fmt.Print(" ")
		}
	}

	fmt.Println()
}

// ------------------------------------------------------------
// Формирование каналов из HID.
// ------------------------------------------------------------

func makeChannels(report HIDReport) [16]uint16 {
	var channels [16]uint16

	// Левый стик.
	leftX := report.RX
	leftY := report.Z

	// Правый стик.
	rightX := report.X
	rightY := report.Y

	// CH1..CH4.
	channels[0] = hidToCRSF(leftX)
	channels[1] = hidToCRSF(leftY)
	channels[2] = hidToCRSF(rightX)
	channels[3] = hidToCRSF(rightY)

	// CH5..CH16 = центр.
	for i := 4; i < 16; i++ {
		channels[i] = CRSFCenter
	}

	return channels
}

// ------------------------------------------------------------
// Основная программа.
// ------------------------------------------------------------

func main() {
	udpEnabled := flag.Bool("udp-enabled", false, "отправлять CRSF-пакеты на UDP сервер")
	udpAddress := flag.String("udp-address", "127.0.0.1:9000", "адрес UDP сервера")
	flag.Parse()

	// --------------------------------------------------------
	// Открываем TX12 HID.
	// --------------------------------------------------------

	devices := hid.Enumerate(VID, PID)

	if len(devices) == 0 {
		log.Fatal("TX12 не найден")
	}

	fmt.Println("Найдено HID устройств:", len(devices))

	for i, device := range devices {
		fmt.Printf(
			"[%d] %s - %s\n",
			i,
			device.Manufacturer,
			device.Product,
		)
	}

	d, err := devices[0].Open()
	if err != nil {
		log.Fatal("не удалось открыть TX12: ", err)
	}
	defer d.Close()

	// --------------------------------------------------------
	// Открываем UART YP-05.
	// --------------------------------------------------------

	mode := &serial.Mode{
		BaudRate: UARTBaudRate,
		DataBits: 8,
		Parity:   serial.NoParity,
		StopBits: serial.OneStopBit,
	}

	uart, err := serial.Open(UARTPort, mode)
	if err != nil {
		log.Fatal(
			"не удалось открыть UART ",
			UARTPort,
			": ",
			err,
		)
	}
	defer uart.Close()

	var udpSender *udpoutput.Sender
	if *udpEnabled {
		udpSender, err = udpoutput.New(*udpAddress)
		if err != nil {
			log.Fatal("не удалось подключить UDP модуль: ", err)
		}
		defer udpSender.Close()
	}

	// --------------------------------------------------------
	// Информация.
	// --------------------------------------------------------

	fmt.Println()
	fmt.Println("TX12 подключен")
	fmt.Println()

	fmt.Println("UART:")
	fmt.Println("  Port       :", UARTPort)
	fmt.Println("  Baud       :", UARTBaudRate)
	fmt.Println("  Format     : 8N1")
	fmt.Println()

	fmt.Println("CRSF bridge:")
	fmt.Println("  CH1 = Left X  = Rx")
	fmt.Println("  CH2 = Left Y  = Z")
	fmt.Println("  CH3 = Right X = X")
	fmt.Println("  CH4 = Right Y = Y")
	fmt.Println("  CH5..CH16 = 992")
	fmt.Println()

	fmt.Println("UDP:")
	if udpSender == nil {
		fmt.Println("  Disabled")
	} else {
		fmt.Println("  Enabled    :", *udpAddress)
	}
	fmt.Println()

	fmt.Println("CRSF:")
	fmt.Println("  Frame size :", CRSFFrameSize)
	fmt.Println("  Payload    :", CRSFPayloadSize)
	fmt.Println("  Channels   : 16 × 11 bit")
	fmt.Println("  Type       : 0x16")
	fmt.Println("  Address    : 0xC8")
	fmt.Println("  CRC        : CRC-8/D5")
	fmt.Println("  Rate       : 200 Hz")
	fmt.Println()

	fmt.Println("Запуск...")
	fmt.Println()

	// --------------------------------------------------------
	// HID buffer.
	// --------------------------------------------------------

	buf := make([]byte, 64)

	// --------------------------------------------------------
	// Последние полученные каналы.
	//
	// HID и UART работают независимо:
	//
	// HID получает управление,
	// UART отправляет его стабильно каждые 5 ms.
	// --------------------------------------------------------

	var channels [16]uint16

	// Начальное состояние:
	// все каналы по центру.
	for i := 0; i < 16; i++ {
		channels[i] = CRSFCenter
	}

	// --------------------------------------------------------
	// Канал HID читаем в отдельной goroutine.
	// --------------------------------------------------------

	hidReports := make(chan HIDReport, 1)
	hidErrors := make(chan error, 1)

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

			// Копируем report, потому что buf переиспользуется.
			reportData := make([]byte, n)
			copy(reportData, buf[:n])

			report := parseReport(reportData)

			select {
			case hidReports <- report:
			default:
				// Если UART ещё не обработал предыдущий report,
				// старый report можно заменить следующим.
				select {
				case <-hidReports:
				default:
				}

				hidReports <- report
			}
		}
	}()

	// --------------------------------------------------------
	// UART ticker = 200 Hz.
	// --------------------------------------------------------

	ticker := time.NewTicker(CRSFInterval)
	defer ticker.Stop()

	var frameCounter uint64

	for {
		select {

		// ----------------------------------------------------
		// Новые данные от TX12.
		// ----------------------------------------------------

		case report := <-hidReports:

			channels = makeChannels(report)

		// ----------------------------------------------------
		// Ошибка HID.
		// ----------------------------------------------------

		case err := <-hidErrors:

			log.Fatal("ошибка HID: ", err)

		// ----------------------------------------------------
		// Каждые 5 ms отправляем CRSF.
		// ----------------------------------------------------

		case <-ticker.C:

			frame := makeCRSF(channels)

			// ------------------------------------------------
			// Проверяем frame перед отправкой.
			// ------------------------------------------------

			frameErr := validateCRSF(frame)

			if frameErr != nil {
				log.Fatal("FRAME CHECK FAIL: ", frameErr)
			}

			packingErr := validatePacking(channels, frame)

			if packingErr != nil {
				log.Fatal("PACK CHECK FAIL: ", packingErr)
			}

			// ------------------------------------------------
			// Отправляем frame отдельно в UART и UDP.
			// ------------------------------------------------

			n, err := sendUART(uart, frame)
			if err != nil {
				log.Fatal(err)
			}

			if err := sendUDP(udpSender, frame); err != nil {
				log.Fatal(err)
			}

			frameCounter++

			// ------------------------------------------------
			// Выводим информацию не на каждый пакет,
			// а примерно 10 раз в секунду,
			// чтобы терминал не превратился в поток.
			// ------------------------------------------------

			if frameCounter%20 == 0 {

				leftX := channels[0]
				leftY := channels[1]
				rightX := channels[2]
				rightY := channels[3]

				fmt.Printf(
					"LEFT CRSF : X=%4d Y=%4d   "+
						"RIGHT CRSF: X=%4d Y=%4d\n",
					leftX,
					leftY,
					rightX,
					rightY,
				)

				printChannels(channels)
				printPayload(frame)
				printCRSF(frame)

				fmt.Printf(
					"UART      : %d bytes sent | CRC=0x%02X | frame #%d\n",
					n,
					frame[25],
					frameCounter,
				)
				if udpSender != nil {
					fmt.Println("UDP       : packet sent")
				}

				fmt.Println(
					"------------------------------------------------------------",
				)
			}
		}
	}
}
