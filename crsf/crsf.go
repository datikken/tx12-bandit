package crsf

const (
	Sync                   = 0xC8
	FrameTypeChannelsPacked = 0x16

	ChannelMin = 172
	ChannelMid = 992
	ChannelMax = 1811
)

// CRC8-DVB-S2, polynomial 0xD5.
func CRC8(data []byte) byte {
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

// Pack 16 channels, 11 bits each = 176 bits = 22 bytes.
func PackChannels(ch []uint16) [22]byte {
	var out [22]byte

	var bitPos uint

	for i := 0; i < 16; i++ {
		v := uint32(ch[i] & 0x07FF)

		for bit := uint(0); bit < 11; bit++ {
			if v&(1<<bit) != 0 {
				out[bitPos/8] |= 1 << (bitPos % 8)
			}

			bitPos++
		}
	}

	return out
}

// MakeRCFrame creates a CRSF RC_CHANNELS_PACKED frame.
//
// C8 18 16 [22-byte payload] [CRC]
func MakeRCFrame(ch []uint16) []byte {
	packed := PackChannels(ch)

	// length = type(1) + payload(22) + CRC(1) = 24 = 0x18
	frame := make([]byte, 0, 26)

	frame = append(frame, Sync)
	frame = append(frame, 24)
	frame = append(frame, FrameTypeChannelsPacked)
	frame = append(frame, packed[:]...)

	// CRC is calculated over TYPE + PAYLOAD.
	frame = append(frame, CRC8(frame[2:]))

	return frame
}
