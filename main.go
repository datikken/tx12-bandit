package main

import (
	"bytes"
	"flag"
	"fmt"
	"log"
	"strings"
	"time"

	"github.com/karalabe/hid"
	"go.bug.st/serial"
)

const (
	Vid = 0x1209
	Pid = 0x4F54

	ReportSize = 19

	AxisMin = 0
	AxisMax = 2047

	CRSFAddress   = 0xC8
	CRSFFrameType = 0x16
	CRSFFrameLen  = 26

	CRSFChannels = 16

	CRSFMin    = 172
	CRSFCenter = 992
	CRSFMax    = 1811

	DefaultPort = "/dev/ttyUSB0"
	DefaultBaud = 420000

	// Bandit observed stream.
	ObservedPacketSize = 22
)

type HIDReport struct {
	LeftX  int
	LeftY  int
	RightX int
	RightY int
}

type StreamPacket struct {
	Data      []byte
	Timestamp time.Time
}

type StreamAnalyzer struct {
	buffer []byte

	packetSize int

	aligned bool
	offset  int

	packets int
	changes int

	lastPacket []byte
	firstPacket []byte

	firstPacketTime time.Time
	lastPacketTime  time.Time

	unique [256]bool
	bytes  int

	periodHits int

	crsfFrames int
}

func NewStreamAnalyzer(packetSize int) *StreamAnalyzer {
	return &StreamAnalyzer{
		packetSize: packetSize,
		buffer:     make([]byte, 0, 64*1024),
	}
}

func (a *StreamAnalyzer) Add(data []byte) []StreamPacket {
	if len(data) == 0 {
		return nil
	}

	a.bytes += len(data)

	for _, b := range data {
		a.unique[int(b)] = true
	}

	a.buffer = append(a.buffer, data...)

	// Prevent unlimited growth while retaining enough data
	// for synchronization.
	if len(a.buffer) > 128*1024 {
		a.buffer = append([]byte(nil), a.buffer[len(a.buffer)-64*1024:]...)
		a.aligned = false
	}

	if !a.aligned {
		if !a.findAlignment() {
			return nil
		}
	}

	var packets []StreamPacket

	for len(a.buffer) >= a.packetSize {
		packet := append([]byte(nil), a.buffer[:a.packetSize]...)
		a.buffer = a.buffer[a.packetSize:]

		now := time.Now()

		if a.packets == 0 {
			a.firstPacket = append([]byte(nil), packet...)
			a.firstPacketTime = now
		} else if !bytes.Equal(packet, a.lastPacket) {
			a.changes++
		}

		a.lastPacket = append(a.lastPacket[:0], packet...)
		a.lastPacketTime = now
		a.packets++

		if isCRSF(packet) {
			a.crsfFrames++
		}

		packets = append(packets, StreamPacket{
			Data:      packet,
			Timestamp: now,
		})
	}

	return packets
}

// findAlignment attempts to determine the 22-byte period.
//
// We do not assume that Read() boundaries correspond to packets.
// Instead, we search for an offset where the following 22-byte
// blocks repeat.
func (a *StreamAnalyzer) findAlignment() bool {
	if len(a.buffer) < a.packetSize*4 {
		return false
	}

	maxOffset := len(a.buffer) - a.packetSize*4

	for offset := 0; offset <= maxOffset; offset++ {
		base := a.buffer[offset : offset+a.packetSize]

		match := true

		for block := 1; block < 4; block++ {
			start := offset + block*a.packetSize
			end := start + a.packetSize

			if !bytes.Equal(base, a.buffer[start:end]) {
				match = false
				break
			}
		}

		if match {
			if offset > 0 {
				a.buffer = append([]byte(nil), a.buffer[offset:]...)
			}

			a.offset = offset
			a.aligned = true
			a.periodHits++

			return true
		}
	}

	// Exact repetition was not found yet.
	//
	// Look for a possible period by comparing bytes separated
	// by 22 positions. This handles a stream where the packet
	// may occasionally change.
	if len(a.buffer) >= a.packetSize*6 {
		for offset := 0; offset < a.packetSize; offset++ {
			matches := 0
			total := 0

			for i := offset; i+a.packetSize < len(a.buffer); i++ {
				total++

				if a.buffer[i] == a.buffer[i+a.packetSize] {
					matches++
				}
			}

			if total > 40 && float64(matches)/float64(total) > 0.98 {
				if offset > 0 {
					a.buffer = append([]byte(nil), a.buffer[offset:]...)
				}

				a.offset = offset
				a.aligned = true
				a.periodHits++

				return true
			}
		}
	}

	// Keep only the tail that can still form an alignment.
	if len(a.buffer) > a.packetSize*3 {
		keep := a.packetSize * 3
		a.buffer = append([]byte(nil), a.buffer[len(a.buffer)-keep:]...)
	}

	return false
}

func (a *StreamAnalyzer) UniqueBytes() []byte {
	result := make([]byte, 0, 256)

	for i := 0; i < 256; i++ {
		if a.unique[i] {
			result = append(result, byte(i))
		}
	}

	return result
}

func (a *StreamAnalyzer) Rate() float64 {
	if a.packets < 2 {
		return 0
	}

	interval := a.lastPacketTime.Sub(a.firstPacketTime)

	if interval <= 0 {
		return 0
	}

	// Correct calculation:
	//
	// packets / seconds
	//
	// Not:
	// time.Second / interval.Seconds()
	return float64(a.packets-1) / interval.Seconds()
}

func (a *StreamAnalyzer) AverageInterval() time.Duration {
	if a.packets < 2 {
		return 0
	}

	interval := a.lastPacketTime.Sub(a.firstPacketTime)

	return interval / time.Duration(a.packets-1)
}

func (a *StreamAnalyzer) FirstPacket() []byte {
	return append([]byte(nil), a.firstPacket...)
}

func (a *StreamAnalyzer) LastPacket() []byte {
	return append([]byte(nil), a.lastPacket...)
}

func (a *StreamAnalyzer) Summary() {
	fmt.Println()
	fmt.Println("--------------------------------------------------")
	fmt.Println("STREAM ANALYZER")
	fmt.Println("--------------------------------------------------")

	fmt.Printf("RX BYTES       : %d\n", a.bytes)
	fmt.Printf("22-BYTE PACKETS: %d\n", a.packets)
	fmt.Printf("PACKET CHANGES : %d\n", a.changes)
	fmt.Printf("CRSF FRAMES    : %d\n", a.crsfFrames)
	fmt.Printf("ALIGNMENT      : %d\n", a.offset)
	fmt.Printf("PERIOD HITS    : %d\n", a.periodHits)

	if interval := a.AverageInterval(); interval > 0 {
		fmt.Printf("PACKET INTERVAL: %s\n", interval)
		fmt.Printf("PACKET RATE    : %.2f Hz\n", a.Rate())
	} else {
		fmt.Printf("PACKET INTERVAL: n/a\n")
		fmt.Printf("PACKET RATE    : n/a\n")
	}

	unique := a.UniqueBytes()

	fmt.Printf("UNIQUE BYTES   : %d\n", len(unique))

	fmt.Printf("BYTE VALUES    :")

	for _, b := range unique {
		fmt.Printf(" %02X", b)
	}

	fmt.Println()

	if len(a.firstPacket) > 0 {
		fmt.Printf("FIRST PACKET   : %s\n", hexString(a.firstPacket))
	}

	if len(a.lastPacket) > 0 {
		fmt.Printf("LAST PACKET    : %s\n", hexString(a.lastPacket))
	}

	fmt.Println("--------------------------------------------------")
}

func hexString(data []byte) string {
	parts := make([]string, len(data))

	for i, b := range data {
		parts[i] = fmt.Sprintf("%02X", b)
	}

	return strings.Join(parts, " ")
}

// ------------------------------------------------------------
// HID
// ------------------------------------------------------------

func u16LE(data []byte, offset int) int {
	if offset+1 >= len(data) {
		return 0
	}

	return int(data[offset]) | int(data[offset+1])<<8
}

func clampAxis(v int) int {
	if v < AxisMin {
		return AxisMin
	}

	if v > AxisMax {
		return AxisMax
	}

	return v
}

func parseReport(data []byte) (HIDReport, bool) {
	if len(data) < ReportSize {
		return HIDReport{}, false
	}

	/*
		Observed TX12 HID layout:

		offset 3  -> left X
		offset 5  -> left Y
		offset 7  -> right X
		offset 9  -> right Y
	*/

	report := HIDReport{
		LeftX:  u16LE(data, 3),
		LeftY:  u16LE(data, 5),
		RightX: u16LE(data, 7),
		RightY: u16LE(data, 9),
	}

	report.LeftX = clampAxis(report.LeftX)
	report.LeftY = clampAxis(report.LeftY)
	report.RightX = clampAxis(report.RightX)
	report.RightY = clampAxis(report.RightY)

	return report, true
}

func hidToCRSF(v int) int {
	v = clampAxis(v)

	return CRSFMin +
		(v*(CRSFMax-CRSFMin))/(AxisMax-AxisMin)
}

func makeChannels(report HIDReport) [CRSFChannels]int {
	var channels [CRSFChannels]int

	channels[0] = hidToCRSF(report.LeftX)
	channels[1] = hidToCRSF(report.LeftY)
	channels[2] = hidToCRSF(report.RightX)
	channels[3] = hidToCRSF(report.RightY)

	for i := 4; i < CRSFChannels; i++ {
		channels[i] = CRSFCenter
	}

	return channels
}

// ------------------------------------------------------------
// CRSF
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

func packChannels(channels [CRSFChannels]int) []byte {
	var payload [22]byte

	var bitOffset uint

	for _, ch := range channels {
		if ch < CRSFMin {
			ch = CRSFMin
		}

		if ch > CRSFMax {
			ch = CRSFMax
		}

		value := uint16(ch)

		for bit := 0; bit < 11; bit++ {
			if value&(1<<bit) != 0 {
				pos := bitOffset + uint(bit)

				payload[pos/8] |= 1 << (pos % 8)
			}
		}

		bitOffset += 11
	}

	return payload[:]
}

func unpackChannels(payload []byte) [CRSFChannels]int {
	var channels [CRSFChannels]int

	if len(payload) < 22 {
		return channels
	}

	var bitOffset uint

	for ch := 0; ch < CRSFChannels; ch++ {
		var value uint16

		for bit := 0; bit < 11; bit++ {
			pos := bitOffset + uint(bit)

			if payload[pos/8]&(1<<(pos%8)) != 0 {
				value |= 1 << bit
			}
		}

		channels[ch] = int(value)

		bitOffset += 11
	}

	return channels
}

func makeCRSF(channels [CRSFChannels]int) []byte {
	payload := packChannels(channels)

	frame := make([]byte, CRSFFrameLen)

	frame[0] = CRSFAddress
	frame[1] = 24
	frame[2] = CRSFFrameType

	copy(frame[3:25], payload)

	frame[25] = crc8(frame[2:25])

	return frame
}

func validateCRSF(frame []byte) bool {
	if len(frame) != CRSFFrameLen {
		return false
	}

	if frame[0] != CRSFAddress {
		return false
	}

	if frame[1] != 24 {
		return false
	}

	if frame[2] != CRSFFrameType {
		return false
	}

	return crc8(frame[2:25]) == frame[25]
}

func validatePacking() bool {
	var channels [CRSFChannels]int

	for i := range channels {
		channels[i] = CRSFCenter
	}

	payload := packChannels(channels)
	unpacked := unpackChannels(payload)

	for i := 0; i < CRSFChannels; i++ {
		if unpacked[i] != channels[i] {
			return false
		}
	}

	return true
}

func isCRSF(data []byte) bool {
	if len(data) != CRSFFrameLen {
		return false
	}

	return validateCRSF(data)
}

// ------------------------------------------------------------
// CRSF stream reader
// ------------------------------------------------------------

type CRSFStreamReader struct {
	buffer []byte
}

func NewCRSFStreamReader() *CRSFStreamReader {
	return &CRSFStreamReader{
		buffer: make([]byte, 0, 1024),
	}
}

func (r *CRSFStreamReader) Feed(data []byte) [][]byte {
	r.buffer = append(r.buffer, data...)

	var result [][]byte

	for {
		if len(r.buffer) < 2 {
			break
		}

		// Search for CRSF address.
		pos := -1

		for i := 0; i < len(r.buffer); i++ {
			if r.buffer[i] == CRSFAddress {
				pos = i
				break
			}
		}

		if pos < 0 {
			if len(r.buffer) > 1 {
				r.buffer = r.buffer[len(r.buffer)-1:]
			}

			break
		}

		if pos > 0 {
			r.buffer = r.buffer[pos:]
		}

		if len(r.buffer) < 2 {
			break
		}

		frameLength := int(r.buffer[1]) + 2

		if frameLength < 4 || frameLength > 64 {
			r.buffer = r.buffer[1:]
			continue
		}

		if len(r.buffer) < frameLength {
			break
		}

		frame := append([]byte(nil), r.buffer[:frameLength]...)
		r.buffer = r.buffer[frameLength:]

		if validateCRSF(frame) {
			result = append(result, frame)
		}
	}

	return result
}

// ------------------------------------------------------------
// UART
// ------------------------------------------------------------

func openUART(portName string, baud int) (serial.Port, error) {
	mode := &serial.Mode{
		BaudRate: baud,
		DataBits: 8,
		StopBits: serial.OneStopBit,
		Parity:   serial.NoParity,
	}

	log.Printf("Opening UART %s @ %d 8N1", portName, baud)

	return serial.Open(portName, mode)
}

// ------------------------------------------------------------
// LOOPBACK
// ------------------------------------------------------------

func runUARTLoopback(port serial.Port) {
	log.Println()
	log.Println("==================================================")
	log.Println(" UART PHYSICAL LOOPBACK")
	log.Println("==================================================")
	log.Println("TX -> RX")
	log.Println("TX12 HID is NOT required")
	log.Println()

	channels := [CRSFChannels]int{}

	for i := range channels {
		channels[i] = CRSFCenter
	}

	frame := makeCRSF(channels)

	log.Printf("TX: %s", hexString(frame))

	n, err := port.Write(frame)

	if err != nil {
		log.Fatalf("UART TX error: %v", err)
	}

	log.Printf("TX bytes: %d", n)

	reader := NewCRSFStreamReader()

	buf := make([]byte, 256)

	deadline := time.Now().Add(2 * time.Second)

	for time.Now().Before(deadline) {
		n, err := port.Read(buf)

		if err != nil {
			log.Fatalf("UART RX error: %v", err)
		}

		if n <= 0 {
			continue
		}

		data := buf[:n]

		log.Printf("RX RAW: %s", hexString(data))

		frames := reader.Feed(data)

		for _, rxFrame := range frames {
			log.Printf("CRSF FRAME: %s", hexString(rxFrame))

			if bytes.Equal(rxFrame, frame) {
				log.Println("RESULT: LOOPBACK CRSF FRAME MATCH")
			} else {
				log.Println("RESULT: CRSF FRAME DIFFERENT")
			}

			return
		}
	}

	log.Println("RESULT: no valid CRSF frame detected")
}

// ------------------------------------------------------------
// RX ONLY
// ------------------------------------------------------------

func runRXOnly(port serial.Port) {
	log.Println()
	log.Println("==================================================")
	log.Println(" BANDIT RX-ONLY STREAM ANALYZER")
	log.Println("==================================================")
	log.Println("TX12 HID is NOT required")
	log.Println("UART TX is NOT used")
	log.Println()
	log.Println("Listening only on Bandit TX")
	log.Println()
	log.Println("Wiring:")
	log.Println("USB-UART RX  <--- Bandit TX")
	log.Println("USB-UART GND ---- Bandit GND")
	log.Println("USB-UART TX      DISCONNECTED")
	log.Println()

	analyzer := NewStreamAnalyzer(ObservedPacketSize)
	crsfReader := NewCRSFStreamReader()

	buf := make([]byte, 4096)

	lastPrintedPacket := []byte(nil)

	start := time.Now()
	nextReport := start.Add(1 * time.Second)

	for {
		n, err := port.Read(buf)

		if err != nil {
			log.Printf("UART RX error: %v", err)
			return
		}

		if n <= 0 {
			if time.Since(start) > 10*time.Second {
				break
			}

			continue
		}

		data := append([]byte(nil), buf[:n]...)

		// Independent CRSF stream parser.
		frames := crsfReader.Feed(data)

		for _, frame := range frames {
			log.Printf("CRSF FRAME DETECTED: %s", hexString(frame))
		}

		// Continuous stream analyzer.
		packets := analyzer.Add(data)

		for _, packet := range packets {
			if lastPrintedPacket == nil {
				lastPrintedPacket = append([]byte(nil), packet.Data...)

				log.Println()
				log.Println("22-BYTE STREAM ALIGNED:")
				log.Printf("PACKET: %s", hexString(packet.Data))
			} else if !bytes.Equal(lastPrintedPacket, packet.Data) {
				log.Println()
				log.Println("22-BYTE PACKET CHANGED:")
				log.Printf("OLD: %s", hexString(lastPrintedPacket))
				log.Printf("NEW: %s", hexString(packet.Data))

				lastPrintedPacket = append(lastPrintedPacket[:0], packet.Data...)
			}
		}

		if time.Now().After(nextReport) {
			log.Println()
			log.Printf(
				"[LIVE] bytes=%d packets=%d changes=%d CRSF=%d rate=%.2f Hz",
				analyzer.bytes,
				analyzer.packets,
				analyzer.changes,
				analyzer.crsfFrames,
				analyzer.Rate(),
			)

			nextReport = time.Now().Add(1 * time.Second)
		}

		if time.Since(start) >= 10*time.Second {
			break
		}
	}

	analyzer.Summary()
}

// ------------------------------------------------------------
// BAUD SCANNER
// ------------------------------------------------------------

type BaudResult struct {
	Baud       int
	Bytes      int
	Packets    int
	Changes    int
	CRSF       int
	Rate       float64
	Interval   time.Duration
	Unique     []byte
	Packet     []byte
	Alignment  int
	PeriodHits int
}

func scanBaud(portName string, baud int, duration time.Duration) BaudResult {
	log.Printf(
		"[SCAN] Testing %d baud for %d ms...",
		baud,
		duration.Milliseconds(),
	)

	port, err := openUART(portName, baud)

	if err != nil {
		log.Printf("[SCAN] %d baud ERROR: %v", baud, err)

		return BaudResult{
			Baud: baud,
		}
	}

	defer port.Close()

	analyzer := NewStreamAnalyzer(ObservedPacketSize)

	crsfReader := NewCRSFStreamReader()

	buf := make([]byte, 4096)

	end := time.Now().Add(duration)

	for time.Now().Before(end) {
		n, err := port.Read(buf)

		if err != nil {
			break
		}

		if n <= 0 {
			continue
		}

		data := buf[:n]

		// Search independently for CRSF.
		frames := crsfReader.Feed(data)

		_ = frames

		analyzer.Add(data)
	}

	return BaudResult{
		Baud:       baud,
		Bytes:      analyzer.bytes,
		Packets:    analyzer.packets,
		Changes:    analyzer.changes,
		CRSF:       analyzer.crsfFrames,
		Rate:       analyzer.Rate(),
		Interval:   analyzer.AverageInterval(),
		Unique:     analyzer.UniqueBytes(),
		Packet:     analyzer.LastPacket(),
		Alignment:  analyzer.offset,
		PeriodHits: analyzer.periodHits,
	}
}

func runBaudScan(portName string) {
	log.Println()
	log.Println("==================================================")
	log.Println("       BANDIT UART BAUD SCANNER")
	log.Println("==================================================")
	log.Printf("PORT: %s", portName)
	log.Println()

	log.Println("IMPORTANT: USB-UART TX must remain disconnected from Bandit RX.")
	log.Println("Only Bandit TX -> USB-UART RX is required.")
	log.Println()

	bauds := []int{
		420000,
		400000,
		460800,
		250000,
		115200,
		57600,
		100000,
		230400,
	}

	results := make([]BaudResult, 0, len(bauds))

	for _, baud := range bauds {
		result := scanBaud(
			portName,
			baud,
			1500*time.Millisecond,
		)

		results = append(results, result)

		unique := len(result.Unique)

		log.Printf(
			"[SCAN] %6d baud | bytes=%7d | aligned-22=%6d | changes=%4d | unique=%2d | CRSF=%3d | rate=%8.2f Hz",
			result.Baud,
			result.Bytes,
			result.Packets,
			result.Changes,
			unique,
			result.CRSF,
			result.Rate,
		)

		if len(result.Packet) > 0 {
			log.Printf(
				"[SCAN]          packet: %s",
				hexString(result.Packet),
			)

			log.Printf(
				"[SCAN]          alignment=%d period-hits=%d",
				result.Alignment,
				result.PeriodHits,
			)
		}

		fmt.Println()
		time.Sleep(250 * time.Millisecond)
	}

	log.Println("==================================================")
	log.Println(" BAUD SCAN FINISHED")
	log.Println("==================================================")

	for _, result := range results {
		log.Printf(
			"%6d baud | bytes=%7d | aligned-22=%6d | changes=%4d | unique=%2d | CRSF=%3d | rate=%8.2f Hz",
			result.Baud,
			result.Bytes,
			result.Packets,
			result.Changes,
			len(result.Unique),
			result.CRSF,
			result.Rate,
		)
	}

	log.Println()

	for _, result := range results {
		if len(result.Packet) == 0 {
			continue
		}

		log.Printf(
			"[RESULT] %d baud representative aligned 22-byte packet:",
			result.Baud,
		)

		log.Printf(
			"         %s",
			hexString(result.Packet),
		)

		log.Printf(
			"[RESULT] alignment=%d period-hits=%d rate=%.2f Hz",
			result.Alignment,
			result.PeriodHits,
			result.Rate,
		)
	}

	log.Println()
	log.Println("[INFO] Scanner does not automatically declare a protocol.")
	log.Println("[INFO] CRSF requires valid CRSF framing + CRC.")
	log.Println("[INFO] 22-byte alignment is only evidence of periodic structure.")
}

// ------------------------------------------------------------
// NORMAL HID -> CRSF -> BANDIT
// ------------------------------------------------------------

func runHIDMode(port serial.Port) {
	log.Println()
	log.Println("==================================================")
	log.Println(" TX12 HID -> CRSF -> BANDIT")
	log.Println("==================================================")

	devices := hid.Enumerate(Vid, Pid)

	log.Printf("Найдено HID устройств: %d", len(devices))

	if len(devices) == 0 {
		log.Println("TX12 не найден")
		return
	}

	for i, device := range devices {
		log.Printf(
			"[%d] %s - %s",
			i,
			device.Manufacturer,
			device.Product,
		)
	}

	device, err := devices[0].Open()

	if err != nil {
		log.Fatalf("не удалось открыть TX12 HID: %v", err)
	}

	defer device.Close()

	log.Println("TX12 подключен")

	buf := make([]byte, ReportSize)

	var txCount uint64

	ticker := time.NewTicker(5 * time.Millisecond)
	defer ticker.Stop()

	var latest HIDReport
	var haveReport bool

	for {
		n, err := device.Read(buf)

		if err != nil {
			log.Printf("HID read error: %v", err)
			return
		}

		if n < ReportSize {
			continue
		}

		report, ok := parseReport(buf[:n])

		if !ok {
			continue
		}

		latest = report
		haveReport = true

		select {
		case <-ticker.C:

			if !haveReport {
				continue
			}

			channels := makeChannels(latest)

			frame := makeCRSF(channels)

			if !validateCRSF(frame) {
				log.Println("ERROR: generated CRSF frame failed validation")
				continue
			}

			n, err := port.Write(frame)

			if err != nil {
				log.Printf("UART TX error: %v", err)
				return
			}

			txCount++

			if txCount%20 == 0 {
				log.Printf(
					"CRSF TX #%d | CH1=%4d CH2=%4d CH3=%4d CH4=%4d",
					txCount,
					channels[0],
					channels[1],
					channels[2],
					channels[3],
				)

				log.Printf(
					"TX -> BANDIT: %s",
					hexString(frame),
				)

				log.Printf(
					"UART TX : %d bytes | CRC=0x%02X",
					n,
					frame[len(frame)-1],
				)
			}

			haveReport = false

		default:
		}
	}
}

// ------------------------------------------------------------
// MAIN
// ------------------------------------------------------------

func main() {
	log.SetFlags(log.LstdFlags | log.Lmicroseconds)

	portName := flag.String(
		"port",
		DefaultPort,
		"UART port",
	)

	baud := flag.Int(
		"baud",
		DefaultBaud,
		"UART baud rate",
	)

	loopback := flag.Bool(
		"loopback",
		false,
		"physical UART loopback test without TX12",
	)

	rxOnly := flag.Bool(
		"rx-only",
		false,
		"listen only to Bandit TX without transmitting",
	)

	baudScan := flag.Bool(
		"baud-scan",
		false,
		"scan UART baud rates",
	)

	flag.Parse()

	if *loopback && *rxOnly {
		log.Fatal("-loopback and -rx-only cannot be used together")
	}

	if *baudScan && (*loopback || *rxOnly) {
		log.Fatal("-baud-scan cannot be combined with -loopback or -rx-only")
	}

	// --------------------------------------------------------
	// BAUD SCAN
	// --------------------------------------------------------

	if *baudScan {
		runBaudScan(*portName)
		return
	}

	// --------------------------------------------------------
	// UART
	// --------------------------------------------------------

	port, err := openUART(*portName, *baud)

	if err != nil {
		log.Fatalf(
			"не удалось открыть UART %s: %v",
			*portName,
			err,
		)
	}

	defer port.Close()

	// --------------------------------------------------------
	// CRSF SELF TEST
	// --------------------------------------------------------

	if !validatePacking() {
		log.Fatal("CRSF packing validation FAILED")
	}

	log.Println("CRSF packing validation OK")

	// --------------------------------------------------------
	// LOOPBACK
	// --------------------------------------------------------

	if *loopback {
		runUARTLoopback(port)
		return
	}

	// --------------------------------------------------------
	// RX ONLY
	// --------------------------------------------------------

	if *rxOnly {
		runRXOnly(port)
		return
	}

	// --------------------------------------------------------
	// NORMAL HID MODE
	// --------------------------------------------------------

	runHIDMode(port)
}