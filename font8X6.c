#include <avr/pgmspace.h>

// one byte is a column
// bit 7 is the bottom

prog_uint8_t font8x6[128][6] =
{
	{ 0x78,0x15,0x14,0x15,0x78,0x00 },	// ASCII -   0    'Ae'
	{ 0x20,0x55,0x54,0x55,0x78,0x00 },	// ASCII -   1    'ae'
	{ 0x38,0x45,0x44,0x45,0x38,0x00 },	// ASCII -   2    'Oe'
	{ 0x30,0x49,0x48,0x49,0x30,0x00 },	// ASCII -   3    'oe'
	{ 0x3c,0x41,0x40,0x41,0x3c,0x00 },	// ASCII -   4    'Ue'
	{ 0x38,0x41,0x40,0x21,0x78,0x00 },	// ASCII -   5    'ue'
	{ 0x7e,0x15,0x15,0x15,0x0a,0x00 },	// ASCII -   6    'sz'
	{ 0x00,0x00,0x00,0x00,0x00,0x00 },	// ASCII -   7	  o-accent
	{ 0x00,0x00,0x00,0x00,0x00,0x00 },	// ASCII -   8    a-circumflex accent
	{ 0x00,0x00,0x00,0x00,0x00,0x00 },	// ASCII -   9    a-tilde
	{ 0x00,0x00,0x00,0x00,0x00,0x00 },	// ASCII -  10  A (not useable)
	{ 0x00,0x00,0x00,0x00,0x00,0x00 },	// ASCII -  11  B c-cedilla
	{ 0x10,0x38,0x54,0x10,0x10,0x1e },	// ASCII -  12  C Enter Symbol
	{ 0x00,0x00,0x00,0x00,0x00,0x00 },	// ASCII -  13  D (not useable)
	{ 0x10,0x10,0x10,0x10,0x10,0x10 },	// ASCII -  14  E hor. line
	{ 0x10,0x10,0x10,0x7c,0x10,0x10 },	// ASCII -  15	F hor. line with tick mark
	{ 0x00,0x00,0x00,0x00,0x00,0x00 },	// ASCII -  16 10 a-accent
	{ 0x00,0x00,0x00,0x00,0x00,0x00 },	// ASCII -  17 11 u-accent
	{ 0x00,0x00,0x00,0x00,0x00,0x00 },	// ASCII -  18 12
	{ 0x00,0x00,0x00,0x00,0x00,0x00 },	// ASCII -  19 13
	{ 0x00,0x00,0x00,0x00,0x00,0x00 },	// ASCII -  20 14
	{ 0x00,0x00,0x00,0x00,0x00,0x00 },	// ASCII -  21 15
	{ 0x04,0x02,0x7f,0x02,0x04,0x00 },	// ASCII -  22 16 Arrow up
	{ 0x10,0x20,0x7f,0x20,0x10,0x00 },	// ASCII -  23 17 Arrow down
	{ 0x10,0x38,0x54,0x10,0x10,0x10 },	// ASCII -  24 18 <-
	{ 0x10,0x10,0x10,0x54,0x38,0x10 },	// ASCII -  25 19 ->
	{ 0x10,0x18,0x1c,0x1c,0x18,0x10 },	// ASCII -  26 1A ^
	{ 0x08,0x18,0x38,0x38,0x18,0x08 },	// ASCII -  27 1B v
	{ 0x00,0x08,0x1c,0x3e,0x7f,0x00 },	// ASCII -  28 1C <
	{ 0x00,0x7f,0x3e,0x1c,0x08,0x00 },	// ASCII -  29 1D >
	{ 0x06,0x09,0x09,0x09,0x06,0x00 },	// ASCII -  30 1E degrees
	{ 0x06,0x49,0x7d,0x49,0x06,0x00 },	// ASCII -  31 1F RC-Tx

	{ 0x00,0x00,0x00,0x00,0x00,0x00 },	// ASCII -  32 20 ' '
	{ 0x00,0x00,0x2f,0x00,0x00,0x00 },	// ASCII -  33 21 '!'
	{ 0x00,0x07,0x00,0x07,0x00,0x00 },	// ASCII -  34 22 '"'
	{ 0x14,0x7f,0x14,0x7f,0x14,0x00 },	// ASCII -  35 23 '#'
	{ 0x24,0x2a,0x6b,0x2a,0x12,0x00 },	// ASCII -  36 24 '$'
	{ 0x23,0x13,0x08,0x64,0x62,0x00 },	// ASCII -  37 25 '%'
	{ 0x36,0x49,0x55,0x22,0x50,0x00 },	// ASCII -  38 26 '&'
	{ 0x00,0x05,0x03,0x00,0x00,0x00 },	// ASCII -  39 27 '''
	{ 0x00,0x1c,0x22,0x41,0x00,0x00 },	// ASCII -  40 28 '('
	{ 0x00,0x41,0x22,0x1c,0x00,0x00 },	// ASCII -  41 29 ')'
	{ 0x14,0x08,0x3e,0x08,0x14,0x00 },	// ASCII -  42 2a '*'
	{ 0x08,0x08,0x3e,0x08,0x08,0x00 },	// ASCII -  43 2b '+'
	{ 0x00,0x50,0x30,0x00,0x00,0x00 },	// ASCII -  44 2c ','
	{ 0x08,0x08,0x08,0x08,0x08,0x00 },	// ASCII -  45 2d '-'
	{ 0x00,0x60,0x60,0x00,0x00,0x00 },	// ASCII -  46 2e '.'
	{ 0x20,0x10,0x08,0x04,0x02,0x00 },	// ASCII -  47 2f '/'
	{ 0x3e,0x51,0x49,0x45,0x3e,0x00 },	// ASCII -  48 30 '0'
	{ 0x00,0x42,0x7f,0x40,0x00,0x00 },	// ASCII -  49 31 '1'
	{ 0x42,0x61,0x51,0x49,0x46,0x00 },	// ASCII -  50 32 '2'
	{ 0x21,0x41,0x45,0x4b,0x31,0x00 },	// ASCII -  51 33 '3'
	{ 0x18,0x14,0x12,0x7f,0x10,0x00 },	// ASCII -  52 34 '4'
	{ 0x27,0x45,0x45,0x45,0x39,0x00 },	// ASCII -  53 35 '5'
	{ 0x3c,0x4a,0x49,0x49,0x30,0x00 },	// ASCII -  54 36 '6'
	{ 0x03,0x01,0x71,0x09,0x07,0x00 },	// ASCII -  55 37 '7'
	{ 0x36,0x49,0x49,0x49,0x36,0x00 },	// ASCII -  56 38 '8'
	{ 0x06,0x49,0x49,0x29,0x1e,0x00 },	// ASCII -  57 39 '9'
	{ 0x00,0x36,0x36,0x00,0x00,0x00 },	// ASCII -  58 3a ':'
	{ 0x00,0x56,0x36,0x00,0x00,0x00 },	// ASCII -  59 3b ';'
	{ 0x08,0x14,0x22,0x41,0x00,0x00 },	// ASCII -  60 3c '<'
	{ 0x14,0x14,0x14,0x14,0x14,0x00 },	// ASCII -  61 3d '='
	{ 0x00,0x41,0x22,0x14,0x08,0x00 },	// ASCII -  62 3e '>'
	{ 0x02,0x01,0x51,0x09,0x06,0x00 },	// ASCII -  63 3f '?'
	{ 0x32,0x49,0x79,0x41,0x3e,0x00 },	// ASCII -  64 40 '@'
	{ 0x7e,0x11,0x11,0x11,0x7e,0x00 },	// ASCII -  65 41 'A'
	{ 0x7f,0x49,0x49,0x49,0x36,0x00 },	// ASCII -  66 42 'B'
	{ 0x3e,0x41,0x41,0x41,0x22,0x00 },	// ASCII -  67 43 'C'
	{ 0x7f,0x41,0x41,0x22,0x1c,0x00 },	// ASCII -  68 44 'D'
	{ 0x7f,0x49,0x49,0x49,0x41,0x00 },	// ASCII -  69 45 'E'
	{ 0x7f,0x09,0x09,0x09,0x01,0x00 },	// ASCII -  70 46 'F'
	{ 0x3e,0x41,0x49,0x49,0x7a,0x00 },	// ASCII -  71 47 'G'
	{ 0x7f,0x08,0x08,0x08,0x7f,0x00 },	// ASCII -  72 48 'H'
	{ 0x00,0x41,0x7f,0x41,0x00,0x00 },	// ASCII -  73 49 'I'
	{ 0x20,0x40,0x41,0x3f,0x01,0x00 },	// ASCII -  74 4a 'J'
	{ 0x7f,0x08,0x14,0x22,0x41,0x00 },	// ASCII -  75 4b 'K'
	{ 0x7f,0x40,0x40,0x40,0x40,0x00 },	// ASCII -  76 4c 'L'
	{ 0x7f,0x02,0x0c,0x02,0x7f,0x00 },	// ASCII -  77 4d 'M'
	{ 0x7f,0x04,0x08,0x10,0x7f,0x00 },	// ASCII -  78 4e 'N'
	{ 0x3e,0x41,0x41,0x41,0x3e,0x00 },	// ASCII -  79 4f 'O'
	{ 0x7f,0x09,0x09,0x09,0x06,0x00 },	// ASCII -  80 50 'P'
	{ 0x3e,0x41,0x51,0x21,0x5e,0x00 },	// ASCII -  81 51 'Q'
	{ 0x7f,0x09,0x19,0x29,0x46,0x00 },	// ASCII -  82 52 'R'
	{ 0x46,0x49,0x49,0x49,0x31,0x00 },	// ASCII -  83 53 'S'
	{ 0x01,0x01,0x7f,0x01,0x01,0x00 },	// ASCII -  84 54 'T'
	{ 0x3f,0x40,0x40,0x40,0x3f,0x00 },	// ASCII -  85 55 'U'
	{ 0x1f,0x20,0x40,0x20,0x1f,0x00 },	// ASCII -  86 56 'V'
	{ 0x3f,0x40,0x38,0x40,0x3f,0x00 },	// ASCII -  87 57 'W'
	{ 0x63,0x14,0x08,0x14,0x63,0x00 },	// ASCII -  88 58 'X'
	{ 0x07,0x08,0x70,0x08,0x07,0x00 },	// ASCII -  89 59 'Y'
	{ 0x61,0x51,0x49,0x45,0x43,0x00 },	// ASCII -  90 5a 'Z'
	{ 0x7f,0x41,0x41,0x00,0x00,0x00 },	// ASCII -  91 5b '['
	{ 0x02,0x04,0x08,0x10,0x20,0x00 },	// ASCII -  92 5c '\'
	{ 0x00,0x41,0x41,0x7f,0x00,0x00 },	// ASCII -  93 5d ']'
	{ 0x04,0x02,0x01,0x02,0x04,0x00 },	// ASCII -  94 5e '^'
	{ 0x40,0x40,0x40,0x40,0x40,0x00 },	// ASCII -  95 5f '_'
	{ 0x00,0x01,0x02,0x04,0x00,0x00 },	// ASCII -  96 60 '`'
	{ 0x20,0x54,0x54,0x54,0x78,0x00 },	// ASCII -  97 61 'a'
	{ 0x7f,0x48,0x44,0x44,0x38,0x00 },	// ASCII -  98 62 'b'
	{ 0x38,0x44,0x44,0x44,0x20,0x00 },	// ASCII -  99 63 'c'
	{ 0x38,0x44,0x44,0x48,0x7f,0x00 },	// ASCII - 100 64 'd'
	{ 0x38,0x54,0x54,0x54,0x18,0x00 },	// ASCII - 101 65 'e'
	{ 0x08,0x7e,0x09,0x01,0x02,0x00 },	// ASCII - 102 66 'f'
	{ 0x0c,0x52,0x52,0x52,0x3e,0x00 },	// ASCII - 103 67 'g'
	{ 0x7f,0x08,0x04,0x04,0x78,0x00 },	// ASCII - 104 68 'h'
	{ 0x00,0x44,0x7d,0x40,0x00,0x00 },	// ASCII - 105 69 'i'
	{ 0x20,0x40,0x44,0x3d,0x00,0x00 },	// ASCII - 106 6a 'j'
	{ 0x7f,0x10,0x28,0x44,0x00,0x00 },	// ASCII - 107 6b 'k'
	{ 0x00,0x41,0x7f,0x40,0x00,0x00 },	// ASCII - 108 6c 'l'
	{ 0x7c,0x04,0x18,0x04,0x78,0x00 },	// ASCII - 109 6d 'm'
	{ 0x7c,0x08,0x04,0x04,0x78,0x00 },	// ASCII - 110 6e 'n'
	{ 0x38,0x44,0x44,0x44,0x38,0x00 },	// ASCII - 111 6f 'o'
	{ 0x7c,0x14,0x14,0x14,0x08,0x00 },	// ASCII - 112 70 'p'
	{ 0x08,0x14,0x14,0x18,0x7c,0x00 },	// ASCII - 113 71 'q'
	{ 0x7c,0x08,0x04,0x04,0x08,0x00 },	// ASCII - 114 72 'r'
	{ 0x48,0x54,0x54,0x54,0x20,0x00 },	// ASCII - 115 73 's'
	{ 0x04,0x3f,0x44,0x40,0x20,0x00 },	// ASCII - 116 74 't'
	{ 0x3c,0x40,0x40,0x20,0x7c,0x00 },	// ASCII - 117 75 'u'
	{ 0x1c,0x20,0x40,0x20,0x1c,0x00 },	// ASCII - 118 76 'v'
	{ 0x3c,0x40,0x38,0x40,0x3c,0x00 },	// ASCII - 119 77 'w'
	{ 0x44,0x28,0x10,0x28,0x44,0x00 },	// ASCII - 120 78 'x'
	{ 0x0c,0x50,0x50,0x50,0x3c,0x00 },	// ASCII - 121 79 'y'
	{ 0x44,0x64,0x54,0x4c,0x44,0x00 },	// ASCII - 122 7a 'z'
	{ 0x00,0x08,0x36,0x41,0x00,0x00 },	// ASCII - 123 7b '{'
	{ 0x00,0x00,0x7f,0x00,0x00,0x00 },	// ASCII - 124 7c '|'
	{ 0x00,0x41,0x36,0x08,0x00,0x00 },	// ASCII - 125 7d '}'
	{ 0x08,0x08,0x2a,0x1c,0x08,0x00 },	// ASCII - 126 7e ->
	{ 0x08,0x1c,0x2a,0x08,0x08,0x00 },	// ASCII - 127 7f <-
};
