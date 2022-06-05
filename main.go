/*
   STMLoader can communicate with STM32xxx builtin UART bootloaders to read and write RAM and Flash and execute programs.
   See
   	AN2606 Application note STM32 microcontroller system memory boot mode
   	AN3155 Application note USART protocol used in the STM32 bootloader

   - get, getv, getid
   - read addr [len]
   - write addr val
   - erase
   - flash file.hex
   - go addr
   - copy remaining serial to stdout

*/
package main

import (
	"bufio"
	"encoding/hex"
	"errors"
	"flag"
	"fmt"
	"io"
	"log"
	"os"
	"time"

	"github.com/pkg/term"
	"github.com/pkg/term/termios"
	"golang.org/x/sys/unix"
)

var (
	fDev    = flag.String("p", "/dev/cu.XXXX", "serial port connected to STM32 chip")
	fBaud   = flag.Int("b", 115200, "baudrate to use")
	fCat    = flag.Bool("c", false, "copy port to stdout after all commands")
	fFile   = flag.String("f", "", "hex file to load or verify")
	fNoload = flag.Bool("n", false, "Don't load, only verify")
)

var (
	commands  []byte
	hexsegs   *Segment
	startaddr uint32 = 0x08000000
)

func main() {

	flag.Parse()

	var err error

	if *fFile != "" {
		var sa uint32
		hexsegs, sa, err = readIhex(*fFile)
		if err != nil {
			log.Fatal(err)
		}
		if sa != 0xffffffff {
			startaddr = sa
		}
	}

	dev, err := term.Open(*fDev,
		term.RawMode,
		term.Speed(*fBaud),
		term.ReadTimeout(500*time.Millisecond),
		term.SetAttr(func(attr *unix.Termios) uintptr {
			attr.Cflag |= unix.PARENB
			return termios.TCSAFLUSH
		}))

	if err != nil {
		log.Fatalf("opening term(%s): %v", *fDev, err)
	}

	if err := connect(dev); err != nil {
		log.Fatal(err)
	}

	cmds, err := cmdGet(dev)
	if err != nil || len(cmds) < 1 {
		log.Fatal("cmd GET: ", err)
	}

	log.Printf("Connected to version %d.%d", cmds[0]>>4, cmds[0]&0xf)
	commands = cmds[1:]
	log.Printf("Commands: % x", commands)

	if false {
		buf, err := cmdGetV(dev)
		log.Printf("cmd GETV: %x (%v)", buf, err)
	}

	buf, err := cmdGetID(dev)
	if err != nil || len(buf) < 2 {
		log.Fatal("cmd GETID: ", err)
	}
	log.Printf("Product ID: %02x%02x", buf[0], buf[1])

	if true {
		buf, err = cmdRead(dev, 0x1FFF7800, 0x30)
		if err != nil || len(buf) < 0x30 {
			log.Fatal("cmd READ optionbytes: ", err)
		}
		// hexdump, todo: parse for device
		for i := 0; i+7 < len(buf); i += 8 {
			fmt.Printf("%08x: %02x %02x %02x %02x  %02x %02x %02x %02x\n", 0x1FFF7800+i, buf[i+7], buf[i+6], buf[i+5], buf[i+4], buf[i+3], buf[i+2], buf[i+1], buf[i])
		}
	}

	if hexsegs != nil {
		if !*fNoload {
			if err := cmdBulkErase(dev); err != nil {
				log.Fatal("cmd BulkErase: ", err)
			}
			for s := hexsegs; s != nil; s = s.Next {
				if err := writeMem(dev, s.Address, s.Data); err != nil {
					log.Fatalf("Writing %d bytes at 0x%08x: %v", len(s.Data), s.Address, err)
				}
				log.Printf("Wrote %d bytes at 0x%08x", len(s.Data), s.Address)
			}
		}

		for s := hexsegs; s != nil; s = s.Next {
			if buf, err := readMem(dev, s.Address, len(s.Data)); err != nil {
				log.Fatalf("Reading back %d bytes at 0x%08x: %v", len(s.Data), s.Address, err)
			} else {
				for i, v := range s.Data {
					if v != buf[i] {
						log.Printf("Verification failed at address 0x%08x. Expected %02x, found %02x", s.Address+uint32(i), v, buf[i])
					}
				}
			}
		}

	}

	log.Printf("Go to 0x%08x...", startaddr)
	if err := cmdGo(dev, startaddr); err != nil {
		log.Println("cmd GO: ", err)
	}

	// dev.SetOption(term.SetAttr(func(attr *unix.Termios) uintptr {
	// 	attr.Cflag &^= unix.PARENB
	// 	return termios.TCSANOW
	// }))
	if *fCat {
		dev.SetReadTimeout(0)
		for {
			var buf [1024]byte
			n, err := dev.Read(buf[:])
			//		fmt.Printf("% x\n", buf[:n])
			os.Stdout.Write(buf[:n])
			if err != nil {
				log.Fatal(err)
			}
		}
	}
}

var (
	errNAK     = errors.New("NAK")
	errTimeout = errors.New("Timeout")
	errBadReq  = errors.New("Bad Request")
)

func getChar(r io.Reader) int {
	var buf [1]byte
	n, _ := r.Read(buf[:])
	//	log.Printf("read %d %v %x", n, err, buf[0])
	if n == 1 {
		return int(buf[0])
	}
	return -1 // timeout
}

func getAck(r io.Reader) error {
	v := getChar(r)
	switch v {
	case 0x79:
		return nil
	case 0x1f:
		return errNAK
	case -1:
		return errTimeout
	}
	return fmt.Errorf("expected ACK, got 0x%x (%c)", v, v)
}

// connect by sending 0x7f, wait for 0x79
func connect(rw io.ReadWriter) error {
	for i := 0; i < 20; i++ { // 20 * 500ms, so 10s.
		if _, err := rw.Write([]byte{0x7f}); err != nil {
			log.Fatal("poking bootloader: ", err)
		}
		switch getChar(rw) {
		case 0x1f:
			log.Println("already connected.")
			return nil
		case 0x79:
			return nil
		}
	}
	return errTimeout
}

func sendBytes(rw io.ReadWriter, chk byte, cmd ...byte) error {
	for _, v := range cmd {
		chk ^= v
	}
	cmd = append(cmd, chk)
	if _, err := rw.Write(cmd); err != nil {
		return err
	}

	return getAck(rw)
}

func sendCmd(rw io.ReadWriter, cmd ...byte) error  { return sendBytes(rw, 0xff, cmd...) }
func sendData(rw io.ReadWriter, cmd ...byte) error { return sendBytes(rw, 0x00, cmd...) }

func cmdGet(rw io.ReadWriter) ([]byte, error) {
	if err := sendCmd(rw, 0x00); err != nil {
		return nil, err
	}

	sz := getChar(rw)
	if sz < 0 {
		return nil, errTimeout
	}

	buf := make([]byte, sz+1)
	if _, err := rw.Read(buf); err != nil {
		return buf, err
	}
	return buf, getAck(rw)
}

func cmdGetV(rw io.ReadWriter) ([]byte, error) {
	if err := sendCmd(rw, 0x01); err != nil {
		return nil, err
	}

	buf := make([]byte, 3)
	if _, err := rw.Read(buf); err != nil {
		return buf, err
	}
	return buf, getAck(rw)
}

func cmdGetID(rw io.ReadWriter) ([]byte, error) {
	if err := sendCmd(rw, 0x02); err != nil {
		return nil, err
	}

	sz := getChar(rw)
	if sz < 0 {
		return nil, errTimeout
	}

	buf := make([]byte, sz+1)
	if _, err := rw.Read(buf); err != nil {
		return buf, err
	}
	return buf, getAck(rw)
}

func cmdRead(rw io.ReadWriter, addr uint32, sz int) ([]byte, error) {
	if sz < 1 || sz > 256 {
		return nil, errBadReq
	}

	if err := sendCmd(rw, 0x11); err != nil {
		return nil, fmt.Errorf("send cmd %v", err)
	}

	if err := sendData(rw, byte(addr>>24), byte(addr>>16), byte(addr>>8), byte(addr)); err != nil {
		return nil, fmt.Errorf("send addr %v", err)
	}

	if err := sendCmd(rw, byte(sz-1)); err != nil {
		return nil, fmt.Errorf("send size %v", err)
	}

	buf := make([]byte, sz)
	n, err := io.ReadFull(rw, buf)
	if err != nil {
		return buf[:n], fmt.Errorf("read %v", err)
	}

	if n != sz {
		log.Println("Warning short read %d instead of %d", n, sz)
	}

	return buf[:n], nil
}

func cmdGo(rw io.ReadWriter, addr uint32) error {
	if err := sendCmd(rw, 0x21); err != nil {
		return err
	}

	return sendData(rw, byte(addr>>24), byte(addr>>16), byte(addr>>8), byte(addr))
}

func cmdWrite(rw io.ReadWriter, addr uint32, buf []byte) error {
	sz := len(buf)
	if sz < 1 || sz > 256 || (sz%4 != 0) {
		return errBadReq
	}

	if err := sendCmd(rw, 0x31); err != nil {
		return fmt.Errorf("send cmd %v", err)
	}

	if err := sendData(rw, byte(addr>>24), byte(addr>>16), byte(addr>>8), byte(addr)); err != nil {
		return fmt.Errorf("send addr %v", err)
	}

	return sendData(rw, append([]byte{byte(len(buf) - 1)}, buf...)...)
}

func cmdBulkErase(rw io.ReadWriter) error {
	if err := sendCmd(rw, 0x44); err != nil {
		return err
	}

	return sendData(rw, 0xff, 0xff)
}

func readMem(rw io.ReadWriter, addr uint32, sz int) ([]byte, error) {
	var buf []byte
	for i := 0; i < sz/256; i++ {
		b, err := cmdRead(rw, addr+uint32(i*256), 256)
		buf = append(buf, b...)
		if err != nil {
			return buf, err
		}
		os.Stderr.Write([]byte("."))
	}
	if sz%256 != 0 {
		b, err := cmdRead(rw, addr+uint32(256*(sz/256)), sz%256)
		buf = append(buf, b...)
		if err != nil {
			return buf, err
		}
		os.Stderr.Write([]byte(":"))
	}
	return buf, nil
}

func writeMem(rw io.ReadWriter, addr uint32, buf []byte) error {
	for len(buf) > 256 {
		if err := cmdWrite(rw, addr, buf[:256]); err != nil {
			return err
		}
		buf = buf[256:]
		addr += 256
		os.Stderr.Write([]byte("."))
	}
	if len(buf) > 0 {
		if err := cmdWrite(rw, addr, buf); err != nil {
			return err
		}
		os.Stderr.Write([]byte(":"))
	}

	return nil
}

type Segment struct {
	Next    *Segment
	Address uint32
	Data    []byte
}

func readIhex(path string) (seg *Segment, startaddr uint32, err error) {

	startaddr = 0xffffffff

	f, err := os.Open(path)
	if err != nil {
		return seg, startaddr, err
	}
	defer f.Close()
	var baseaddr uint32

	scanner := bufio.NewScanner(f)

	for scanner.Scan() {
		if scanner.Text() == "" {
			continue
		}
		if scanner.Text()[0] != ':' {
			log.Printf("Malformed line %q", scanner.Text())
			continue
		}
		buf, err := hex.DecodeString(scanner.Text()[1:])
		if err != nil {
			log.Println(err)
			continue
		}
		var chk byte
		for _, v := range buf {
			chk += v
		}
		if (chk != 0) || (len(buf) < 4) {
			log.Printf("% x  bad checksum %x\n", buf, chk)
			continue
		}
		if len(buf) < 5+int(buf[0]) {
			log.Printf("% x  invalid len %d != %d\n", buf, buf[0], len(buf)-5)
			continue
		}

		switch buf[3] {
		case 0x00:
			addr := baseaddr + uint32(buf[1])<<8 | uint32(buf[2])
			if (seg == nil) || (seg.Address+uint32(len(seg.Data)) != addr) {
				seg = &Segment{Next: seg, Address: addr}
			}
			seg.Data = append(seg.Data, buf[4:4+buf[0]]...)
		case 0x04:
			if buf[0] != 2 {
				log.Printf("% x  invalid len %d for type 4\n", buf, buf[0])
				continue
			}
			baseaddr = uint32(buf[4])<<24 | uint32(buf[5])<<16
		case 0x05:
			if buf[0] != 4 {
				log.Printf("% x  invalid len %d for type 5\n", buf, buf[0])
				continue
			}
			startaddr = uint32(buf[4])<<24 | uint32(buf[5])<<16 | uint32(buf[6])<<8 | uint32(buf[7])
		}

	}
	if err := scanner.Err(); err != nil {
		log.Fatal(err)
	}
	reverse(&seg)

	return seg, startaddr, err
}

//	for s := seg; s != nil; s = s.Next {
//		fmt.Printf("Seg %08x len %d: % x..\n", s.Address, len(s.Data), s.Data[:10])
//	}

// https://play.golang.org/p/hn9rKOgZl3k
func reverse(pp **Segment) {
	var prev, cur, next *Segment
	for cur = *pp; cur != nil; cur = next {
		next, cur.Next, prev = cur.Next, prev, cur
	}
	*pp = prev
}
