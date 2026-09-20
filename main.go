package main

import (
	"fmt"
	"log"

	"github.com/karalabe/hid"
)

const (
	VID = 0x1209
	PID = 0x4F54

	ReportSize = 19

	// HID axis range согласно Report Descriptor.
	AxisMin = 0
	AxisMax = 2047

	// CRSF RC channel range.
	CRSFMin    = 172
	CRSFCenter = 992
	CRSFMax    = 1811
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
// При HID:
//
// 0    -> 172
// 1024 -> 992
// 2047 -> 1811
//
// Делим диапазон относительно центра, чтобы нейтраль TX12
// корректно попадала примерно в CRSF center = 992.
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
// CRSF CRC-8
//
// Polynomial: 0xD5
// CRC считается от TYPE + PAYLOAD.
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
// CRSF RC Channels Packed
//
// Frame:
//
// 0     ADDRESS      C8
// 1     LENGTH       18
// 2     TYPE         16
// 3..24 PAYLOAD      22 bytes
// 25    CRC
//
// Payload:
// 16 channels × 11 bits = 176 bits = 22 bytes
// ------------------------------------------------------------

func makeCRSF(channels [16]uint16) []byte {
	payload := make([]byte, 22)

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

	// Полный CRSF frame = 26 bytes.
	frame := make([]byte, 26)

	// Address.
	frame[0] = 0xC8

	// Length = TYPE + PAYLOAD + CRC
	// 1 + 22 + 1 = 24 = 0x18.
	frame[1] = 0x18

	// RC Channels Packed.
	frame[2] = 0x16

	// Payload.
	copy(frame[3:25], payload)

	// CRC считается начиная с TYPE.
	frame[25] = crc8(frame[2:25])

	return frame
}

func printButtons(buttons uint32) {
	fmt.Print("Buttons:")

	found := false

	for i := 0; i < 24; i++ {
		if buttons&(1<<i) != 0 {
			fmt.Printf(" %d", i+1)
			found = true
		}
	}

	if !found {
		fmt.Print(" none")
	}

	fmt.Println()
}

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

func printCRSF(frame []byte) {
	fmt.Print("CRSF: ")

	for _, b := range frame {
		fmt.Printf("%02X ", b)
	}

	fmt.Println()
}

func main() {
	devices := hid.Enumerate(VID, PID)

	if len(devices) == 0 {
		log.Fatal("TX12 не найден")
	}

	fmt.Println("Найдено устройств:", len(devices))

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
		log.Fatal(err)
	}
	defer d.Close()

	fmt.Println()
	fmt.Println("TX12 подключен")
	fmt.Println()
	fmt.Println("CRSF bridge:")
	fmt.Println("  CH1 = Left X  = Rx")
	fmt.Println("  CH2 = Left Y  = Z")
	fmt.Println("  CH3 = Right X = X")
	fmt.Println("  CH4 = Right Y = Y")
	fmt.Println("  CH5..CH16 = 992")
	fmt.Println()

	buf := make([]byte, 64)

	for {
		n, err := d.Read(buf)
		if err != nil {
			log.Fatal(err)
		}

		if n < ReportSize {
			continue
		}

		report := parseReport(buf)

		// ----------------------------------------------------
		// Стики TX12
		// ----------------------------------------------------

		// Левый стик:
		// Horizontal = Rx
		// Vertical   = Z
		leftX := report.RX
		leftY := report.Z

		// Правый стик:
		// Horizontal = X
		// Vertical   = Y
		rightX := report.X
		rightY := report.Y

		// ----------------------------------------------------
		// Формируем 16 CRSF каналов.
		// ----------------------------------------------------

		var channels [16]uint16

		// CH1-CH4.
		channels[0] = hidToCRSF(leftX)
		channels[1] = hidToCRSF(leftY)
		channels[2] = hidToCRSF(rightX)
		channels[3] = hidToCRSF(rightY)

		// CH5-CH16 пока центр.
		for i := 4; i < 16; i++ {
			channels[i] = CRSFCenter
		}

		// ----------------------------------------------------
		// Формируем настоящий CRSF frame.
		// ----------------------------------------------------

		frame := makeCRSF(channels)

		// ----------------------------------------------------
		// Вывод.
		// ----------------------------------------------------

		fmt.Printf(
			"LEFT : X=%4d Y=%4d   "+
				"RIGHT: X=%4d Y=%4d\n",
			leftX,
			leftY,
			rightX,
			rightY,
		)

		printChannels(channels)

		printCRSF(frame)

		printButtons(report.Buttons)

		fmt.Println()
	}
}
