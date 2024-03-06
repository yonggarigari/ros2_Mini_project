## 1. `pyserial` 라이브러리를 이용한 시리얼통신코드 작성
ros2 와 stm32가 시리얼 통신을 하기 위해 몇 가지 설정해줘야 한다.
<br>
- com.c : 시리얼 통신을 통해 데이터를 주고 받는 코드
```c
#include <termios.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <unistd.h>
#include <sys/signal.h>
#include <sys/types.h>
#include <sys/ioctl.h>
#include <fcntl.h>
#include <errno.h>

int transfer_byte(int from, int to, int is_control);

typedef struct {char *name; int flag; } speed_spec;


void print_status(int fd) {
	int status;
	unsigned int arg;
	status = ioctl(fd, TIOCMGET, &arg);
	fprintf(stderr, "[STATUS]: ");
	if(arg & TIOCM_RTS) fprintf(stderr, "RTS ");
	if(arg & TIOCM_CTS) fprintf(stderr, "CTS ");
	if(arg & TIOCM_DSR) fprintf(stderr, "DSR ");
	if(arg & TIOCM_CAR) fprintf(stderr, "DCD ");
	if(arg & TIOCM_DTR) fprintf(stderr, "DTR ");
	if(arg & TIOCM_RNG) fprintf(stderr, "RI ");
	fprintf(stderr, "\r\n");
}

int main(int argc, char *argv[])
{
	int comfd;
	struct termios oldtio, newtio;       //place for old and new port settings for serial port
	struct termios oldkey, newkey;       //place tor old and new port settings for keyboard teletype
	char *devicename = argv[1];
	int need_exit = 0;
	speed_spec speeds[] =
	{
		{"1200", B1200},
		{"2400", B2400},
		{"4800", B4800},
		{"9600", B9600},
		{"19200", B19200},
		{"38400", B38400},
		{"57600", B57600},
		{"115200", B115200},
		{NULL, 0}
	};
	int speed = B9600;

	if(argc < 2) {
		fprintf(stderr, "example: %s /dev/ttyS0 [115200]\n", argv[0]);
		exit(1);
	}

	comfd = open(devicename, O_RDWR | O_NOCTTY | O_NONBLOCK);
	if (comfd < 0)
	{
		perror(devicename);
		exit(-1);
	}

	if(argc > 2) {	
		speed_spec *s;
		for(s = speeds; s->name; s++) {
			if(strcmp(s->name, argv[2]) == 0) {
				speed = s->flag;
				fprintf(stderr, "setting speed %s\n", s->name);
				break;
			}
		}
	}

	fprintf(stderr, "Ctrl-Q exit, Ctrl-Y modem lines status\n");

	tcgetattr(STDIN_FILENO,&oldkey);
	newkey.c_cflag = B9600 | CRTSCTS | CS8 | CLOCAL | CREAD;
	newkey.c_iflag = IGNPAR;
	newkey.c_oflag = 0;
	newkey.c_lflag = 0;
	newkey.c_cc[VMIN]=1;
	newkey.c_cc[VTIME]=0;
	tcflush(STDIN_FILENO, TCIFLUSH);
	tcsetattr(STDIN_FILENO,TCSANOW,&newkey);


	tcgetattr(comfd,&oldtio); // save current port settings 
	newtio.c_cflag = speed | CS8 | CLOCAL | CREAD;
	newtio.c_iflag = IGNPAR;
	newtio.c_oflag = 0;
	newtio.c_lflag = 0;
	newtio.c_cc[VMIN]=1;
	newtio.c_cc[VTIME]=0;
	tcflush(comfd, TCIFLUSH);
	tcsetattr(comfd,TCSANOW,&newtio);

	print_status(comfd);

	while(!need_exit) {
		fd_set fds;
		int ret;
		
		FD_ZERO(&fds);
		FD_SET(STDIN_FILENO, &fds);
		FD_SET(comfd, &fds);


		ret = select(comfd+1, &fds, NULL, NULL, NULL);
		if(ret == -1) {
			perror("select");
		} else if (ret > 0) {
			if(FD_ISSET(STDIN_FILENO, &fds)) {
				need_exit = transfer_byte(STDIN_FILENO, comfd, 1);
			}
			if(FD_ISSET(comfd, &fds)) {
				need_exit = transfer_byte(comfd, STDIN_FILENO, 0);
			}
		}
	}

	tcsetattr(comfd,TCSANOW,&oldtio);
	tcsetattr(STDIN_FILENO,TCSANOW,&oldkey);
	close(comfd);

	return 0;
}


int transfer_byte(int from, int to, int is_control) {
	char c;
	int ret;
	do {
		ret = read(from, &c, 1);
	} while (ret < 0 && errno == EINTR);
	if(ret == 1) {
		if(is_control) {
			if(c == '\x11') { // Ctrl-Q
				return -1;
			} else if(c == '\x19') {	// Ctrl-Y
				print_status(to);
				return 0;
			}
		}
		while(write(to, &c, 1) == -1) {
			if(errno!=EAGAIN && errno!=EINTR) { perror("write failed"); break; }
		}
	} else {
		fprintf(stderr, "\nnothing to read. probably port disconnected.\n");
		return -2;
	}
	return 0;
}
```

com.c 파일을 저장하고 컴파일 해준다.

```
gcc -o com com.c
```
이제 시리얼 통신으로 '1' 또는 '0'을 송신하는 파이썬 코드를 작성해보자. 이 작업에는 `pyserial` 라이브러리가 필요하다.

```
pip3 install pyserial
```
파이썬3를 위한 pip가 설치되어 있지 않아 에러가 발생할 경우, 다음 명령으로 설치할 수 있다.
```
sudo apt install python3-pip
```
연결된 stm32의 경로를 확인하기 위해 해당 코드를 입력해준다.

```
ls /dev/tty*
```
stm32의 경로는 `ttyACM0` 이다.
<br>
<br>
`com`을 실행하기 위해선 `./com /dev/ttyACM0 115200` 입력해주면 되는데 이렇게 작성하면 오류가 발생한다. 그 이유는 stm32 장치에 대한 읽기/쓰기 권한이 없기 때문이므로 권한을 부여해주면 오류가 해결된다. 권한을 부여할때는 일시적인 방법과 영구적인 방법이 있다.
<br>
1. 일시적인 방법
```
sudo chmod 666 /dev/ttyACM0
```
2. 영구적인 방법
```
sudo usermod -aG dialout $USER
```

## 2. stm32 제어 패키지 생성


