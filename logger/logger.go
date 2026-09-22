package logger

import (
	"fmt"
	"log"
)

type AppLogger struct {
	logger *log.Logger
}

func NewLogger() *AppLogger {
	return &AppLogger{
		logger: log.Default(),
	}
}

func (l *AppLogger) Info(format string, args ...interface{}) {
	l.logger.Printf("[INFO] "+format, args...)
}

func (l *AppLogger) Error(format string, args ...interface{}) {
	l.logger.Printf("[ERROR] "+format, args...)
}

func (l *AppLogger) Fatal(format string, args ...interface{}) {
	l.logger.Fatalf("[FATAL] "+format, args...)
}

func (l *AppLogger) Print(format string, args ...interface{}) {
	fmt.Printf(format, args...)
}

func (l *AppLogger) Println(args ...interface{}) {
	fmt.Println(args...)
}

func (l *AppLogger) Separator() {
	fmt.Println("------------------------------------------------------------")
}

func (l *AppLogger) Header(title string) {
	fmt.Println()
	fmt.Println("==============================================")
	fmt.Println(title)
	fmt.Println("==============================================")
	fmt.Println()
}

func (l *AppLogger) Channels(channels [16]uint16) {
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

func (l *AppLogger) CRSF(label string, frame []byte) {
	fmt.Printf("%s: ", label)

	for i, b := range frame {
		fmt.Printf("%02X", b)

		if i < len(frame)-1 {
			fmt.Print(" ")
		}
	}

	fmt.Println()
}

func (l *AppLogger) Payload(frame []byte) {
	fmt.Print("PAYLOAD : ")

	for i := 3; i < 25 && i < len(frame); i++ {
		fmt.Printf("%02X", frame[i])

		if i < 24 {
			fmt.Print(" ")
		}
	}

	fmt.Println()
}

var Logger = NewLogger()
