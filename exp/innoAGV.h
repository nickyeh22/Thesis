#ifndef __INNOAGV__
#define __INNOAGV__
#include <winsock2.h>
#include <ws2tcpip.h>
#pragma comment(lib,"ws2_32.lib") //Winsock Library
#define DEF_CAR		    "192.168.1.113"
#define PORT		      20108	//The port on which to listen for incoming data
#define RCV_TIMEOUT   1000
#include <stdint.h>
class innoAGV {
  
  int err;
  int s;
  sockaddr_in sin;
  int sl;
  char server[64];
  unsigned char rbuf[1024];
  int rlen;
  int state;
  
public:
  bool debug;
  innoAGV() {
    debug = false;
	state = 0;
	err = 1;
	sl = sizeof(sin);
	sprintf_s(server, DEF_CAR);
    state = 0;
    rbuf[0] = 0;
    rlen = 0;
    s = 0;
  };
  bool ok() {
    return err == 0;
  }
  int init() {
    static WSADATA wsa;
    if (WSAStartup(MAKEWORD(2, 2), &wsa) != 0) {
	  if(debug) {
		printf("net error : %d\r\n", WSAGetLastError());
	  }
      err = 2;
      return -2;
    }
    if ((s = socket(AF_INET, SOCK_DGRAM, IPPROTO_UDP)) == SOCKET_ERROR) {
      if(debug) {
		printf("net error : %d\r\n", WSAGetLastError());
	  }
      err = 3;
      return -3;
    }
    int timeout = RCV_TIMEOUT; 
    int ret = setsockopt(s, SOL_SOCKET, SO_RCVTIMEO, (char*)&timeout, sizeof(timeout));
    memset((char *)&sin, 0, sizeof(sin));
    sin.sin_family = AF_INET;
    sin.sin_port = htons(PORT);
    inet_pton(AF_INET, DEF_CAR, (void*)&sin.sin_addr.S_un.S_addr);
    err = 0;
    return 0;
  }
  void close() {
    closesocket(s);
    WSACleanup();
  }
  void select(const char* car) {
    memset((char *)&sin, 0, sizeof(sin));
    sin.sin_family = AF_INET;
    sin.sin_port = htons(PORT);
    inet_pton(AF_INET, car, (void*)&sin.sin_addr.S_un.S_addr);
  }
  int send(const char *b, int l) {
    if (err < 0) return err;
    sl = sizeof(sockaddr_in);
    if (sendto(s, b, l, 0, (struct sockaddr *) &sin, sl) == SOCKET_ERROR) {
      if(debug) {
		printf("net error : %d", WSAGetLastError());
	  }
    }
    return 0;
  }
  int reveive() {
    if (err != 0) {
		if(debug) {
			printf("ERROR RECV\r\n");
		}
		return err;
	}
    sl = sizeof(struct sockaddr_in);
    return recvfrom(s, (char*)rbuf, sizeof(rbuf), 0, (sockaddr *)&sin, &sl);
  }
  int ledRedOn(int en) {
    char buf[6];
    buf[0] = 'r';
	buf[1] = en;
    send(buf, 2);
    int l = reveive();
    if (l <= 0) {
		if(debug) {
			printf("NETWORK ERROR\r\n");
		}
      return -5;
    } else if (l == 1) {
      showDebugInfo();
	  return 0;
    } else {
		printf("UNKNOWN ERROR %d %d\r\n", l, rbuf[0]);
	}
    return 1;
  }
  int ledGreenOn(int en) {
    char buf[6];
    buf[0] = 'g';
	buf[1] = en;
    send(buf, 2);
    int l = reveive();
    if (l <= 0) {
		if(debug) {
			printf("NETWORK ERROR\r\n");
		}
      return -5;
    } else if (l == 1) {
      showDebugInfo();
	  return 0;
    }
    return 1;
  }
  int buzzerOn(int en) {
    char buf[6];
    buf[0] = 'u';
	buf[1] = en;
    send(buf, 2);
    int l = reveive();
    if (l <= 0) {
		if(debug) {
			printf("NETWORK ERROR\r\n");
		}
      return -5;
    } else if (l == 1) {
      showDebugInfo();
	  return 0;
    }
    return 1;
  }
  int resetEncoder() {
    char buf[2];
    buf[0] = 'b';
    send(buf, 1);
    int l = reveive();
    if (l <= 0) {
		if(debug) {
			printf("NETWORK ERROR\r\n");
		}
      return -5;
    } else if (l == 1)  {
      showDebugInfo();
	  return 0;
    }
    return 1;
  }
  int ledOff() {
    char buf[6];
    buf[0] = 'o';
    send(buf, 1);
    int l = reveive();
    if (l <= 0) {
		if(debug) {
			printf("NETWORK ERROR\r\n");
		}
      return -5;
    } else if (l == 1) {
      showDebugInfo();
      return 0;
    }
    return 1;
  }
  int enableMotor(int en) {
    char buf[6];
    buf[0] = 'm';
    buf[1] = en;
    send(buf, 2);
    int l = reveive();
    if (l <= 0) {
      return -5;
    } else if (l == 1) {
	  showDebugInfo();
      return 0;
    }
    return 1;
  }
  int setVelDS(int Dir, int Speed) {
    char buf[6];
    buf[0] = 's';
    buf[1] = Dir & 0xff;
    buf[2] = (Dir >>8) & 0xff;
    buf[3] = Speed & 0xff;
    buf[4] = (Speed >> 8) & 0xff;
    send(buf, 5);
    int l = reveive();
    if (l <= 0) {
      return -5;
    } else if (l == 1) {
      showDebugInfo();
      return 0;
    }
    return 1;
  }
  int getLaser(int deg, int& dist) {
	dist = 0;
    char buf[6];
    buf[0] = 'l';
    buf[1] = deg & 0xff;
    buf[2] = (deg >> 8) & 0xff;
    send(buf, 3);
    int l = reveive();
    if (l <= 0) {
      return -5;
    } else if (l == 3) {
      showDebugInfo();
      dist = rbuf[1] | (rbuf[2] << 8);
      return 0;
    }
    return 1;
  }
  int getBattery(double& volt) {
	  volt = -1;
    char buf[6];
    buf[0] = 'B';
    send(buf, 1);
    int l = reveive();
    if (l <= 0) {
      return -5;
    } else if (l == 3) {
      showDebugInfo();
      int16_t volt16 = rbuf[1] | (rbuf[2] << 8);
      volt = volt16/1000.0;
      return 0;
    }
    return 1;
  }
  void showDebugInfo() {
    if((rbuf[0] != 0) && debug) {
		if(rbuf[0] & 1) {
		  printf("LIDAR ERROR\r\n");
		}
		if(rbuf[0] & 2) {
		  printf("MOTOR ERROR\r\n");
		}
		if(rbuf[0] & 4) {
		  printf("SENSOR ERROR\r\n");
		}
		if(rbuf[0] & 0x80) {
		  printf("EMO\r\n");
		}
		if(rbuf[0] & 0x40) {
		  printf("BUMP\r\n");
		}
		if(rbuf[0] & 0x20) {
		  printf("COMMAND ERROR\r\n");
		}
	}
  }
  int getLaser360(int (&dist)[360]) {
    char buf[6];
    buf[0] = 'L';
    send(buf, 1);
    int l = reveive();
    if (l <= 0) {
      return -5;
    } else if (l == 721) {
	  showDebugInfo();
      for (int i = 0; i < 360; i++) {
        dist[i] =  rbuf[2*i+1] | (rbuf[2*i+2] << 8);
      }
	  
      return 0;
    }
    return 1;
  }
  int getVersion(char *v) {

    char buf[6];
    buf[0] = 'v';
    send(buf, 1);
    int l = reveive();
    if (l <= 0) {
      return -5;
    } else if (l >= 6)	{
		showDebugInfo();
        for (int i = 1; i < 6; i++) {
          *v++ = rbuf[i];
        }
		*v = 0;
        return 0;
    }
    return 1;
  }
  int tachInDual(int &L, int &R) {
    char buf[6];
    buf[0] = 'e';
    send(buf, 1);
    int l = reveive();
    if (l <= 0) {
      return -5;
    } else if (l >= 9) {
      showDebugInfo(); 
	  int32_t sL =  rbuf[1] | (rbuf[2] << 8) | (rbuf[3] << 16) | (rbuf[4] << 24);
	  int32_t sR =  rbuf[5] | (rbuf[6] << 8) | (rbuf[7] << 16) | (rbuf[8] << 24);
      L = sL;
      R = sR;
	  return 0;
    }
    return 1;
  }
  
  int get9AxisData(int *angle, int* AngularVelocity, int* Acceleration, int* Magnetic) {
    char buf[6];
    buf[0] = 'a';
    send(buf, 1);
    int l = reveive();
    if (l <= 0) {
      return -5;
    } else if (l >= 25) {
      showDebugInfo(); 
	  int16_t v;
	  v = rbuf[1] | (rbuf[2] << 8);
	  angle[0] = v;
	  v = rbuf[3] | (rbuf[4] << 8);
	  angle[1] = v;
	  v = rbuf[5] | (rbuf[6] << 8);
	  angle[2] = v;

	  v = rbuf[7] | (rbuf[8] << 8);
	  AngularVelocity[0] = v;
	  v = rbuf[9] | (rbuf[10] << 8);
	  AngularVelocity[1] = v;
	  v = rbuf[11] | (rbuf[12] << 8);
	  AngularVelocity[2] = v;

	  v = rbuf[13] | (rbuf[14] << 8);
	  Acceleration[0] = v;
	  v = rbuf[15] | (rbuf[16] << 8);
	  Acceleration[1] = v;
	  v = rbuf[17] | (rbuf[18] << 8);
	  Acceleration[2] = v;


	  v = rbuf[19] | (rbuf[20] << 8);
	  Magnetic[0] = v;
	  v = rbuf[21] | (rbuf[22] << 8);
	  Magnetic[1] = v;
	  v = rbuf[23] | (rbuf[24] << 8);
	  Magnetic[2] = v;

	  return 0;
    }
    return 1;
  }
};
#endif
