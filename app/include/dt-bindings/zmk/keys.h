
#pragma once

#define A 0x04
#define B 0x05
#define C 0x06
#define D 0x07
#define E 0x08
#define F 0x09
#define G 0x0A
#define H 0x0B
#define I 0x0C
#define J 0x0D
#define K 0x0E
#define L 0x0F
#define M 0x10
#define N 0x11
#define O 0x12
#define P 0x13
#define Q 0x14
#define R 0x15
#define S 0x16
#define T 0x17
#define U 0x18
#define V 0x19
#define W 0x1A
#define X 0x1B
#define Y 0x1C
#define Z 0x1D
#define NUM_1 0x1E
#define NUM_2 0x1F
#define NUM_3 0x20
#define NUM_4 0x21
#define NUM_5 0x22
#define NUM_6 0x23
#define NUM_7 0x24
#define NUM_8 0x25
#define NUM_9 0x26
#define NUM_0 0x27
#define RET 0x28
#define ESC 0x29
#define DEL 0x2A
#define BKSP DEL
#define TAB 0x2B
#define SPC 0x2C
#define MINUS 0x2D
#define EQL 0x2E
#define LBKT 0x2F
#define RBKT 0x30
#define FSLH 0x31

#define SCLN 0x33
#define QUOT 0x34
#define GRAV 0x35
#define CMMA 0x36
#define DOT 0x37
#define BSLH 0x38
#define CLCK 0x39
#define F1 0x3A
#define F2 0x3B

#define RARW 0x4F
#define LARW 0x50
#define DARW 0x51
#define UARW 0x52

#define KDIV 0x54
#define KMLT 0x55
#define KMIN 0x56
#define KPLS 0x57

#define GUI 0x65

#define CURU 0xB4

#define LPRN 0xB6
#define RPRN 0xB7
#define LCUR 0xB8
#define RCUR 0xB9

#define CRRT 0xC3
#define PRCT 0xC4
#define LABT 0xC5
#define RABT 0xC6
#define AMPS 0xC7
#define PIPE 0xC9
#define COLN 0xCB
#define HASH 0xCC
#define KSPC 0xCD
#define ATSN 0xCE
#define BANG 0xCF

#define LCTL 0xE0
#define LSFT 0xE1
#define LALT 0xE2
#define LGUI 0xE3
#define RCTL 0xE4
#define RSFT 0xE5
#define RALT 0xE6
#define RGUI 0xE7

#define VOLU 0x80
#define VOLD 0x81

/* The following are select consumer page usages */

#define MNXT 0x100
#define MPRV 0x101
#define MSTP 0x102
#define MJCT 0x103
#define MPLY 0x104
#define MMUT 0x105
#define MVLU 0x106
#define MVLD 0x107

#define MOD_LCTL (1 << 0x00)
#define MOD_LSFT (1 << 0x01)
#define MOD_LALT (1 << 0x02)
#define MOD_LGUI (1 << 0x03)
#define MOD_RCTL (1 << 0x04)
#define MOD_RSFT (1 << 0x05)
#define MOD_RALT (1 << 0x06)
#define MOD_RGUI (1 << 0x07)

#define ZK_IS_CONSUMER(k) (ZK_KEY(k) >= 0x100)
