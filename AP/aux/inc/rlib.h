#ifndef R_LIB_H
#define R_LIB_H

#if 0
#if RLIB_USE_STRLEN
size_t strlen(const char *s);
#endif

#if RLIB_USE_MEMCPY
void *memcpy(void *dest, const void *src, size_t n);
#endif

#if RLIB_USE_MEMSET
void *memset(void *s, int c, size_t n);
#endif
#endif

size_t strlen(const char *s);
void *memcpy(void *dest, const void *src, size_t n);
void *memset(void *s, int c, size_t n);
char *strcpy(char *dst, const char *src);

// #if RLIB_USE_CTYPE

extern char _ctypes[256];

#define RLIB_CTYPE_LOWER 0x01
#define RLIB_CTYPE_UPPER 0x02
#define RLIB_CTYPE_DIGIT 0x04
#define RLIB_CTYPE_SPACE 0x08
#define RLIB_CTYPE_PUNCT 0x10
#define RLIB_CTYPE_CNTRL 0x20
#define RLIB_CTYPE_GRAPH 0x40
#define RLIB_CTYPE_XDIGIT 0x80

INLINE int rlib_isalnum(int c) { return _ctypes[c] & (RLIB_CTYPE_LOWER | RLIB_CTYPE_UPPER | RLIB_CTYPE_DIGIT); }
INLINE int rlib_isalpha(int c) { return _ctypes[c] & (RLIB_CTYPE_LOWER | RLIB_CTYPE_UPPER); }
INLINE int rlib_iscntrl(int c) { return _ctypes[c] & (RLIB_CTYPE_CNTRL); }
INLINE int rlib_isdigit(int c) { return _ctypes[c] & (RLIB_CTYPE_DIGIT); }
INLINE int rlib_isgraph(int c) { return _ctypes[c] & (RLIB_CTYPE_GRAPH); }
INLINE int rlib_islower(int c) { return _ctypes[c] & (RLIB_CTYPE_LOWER); }
INLINE int rlib_isprint(int c) { return _ctypes[c] & (RLIB_CTYPE_GRAPH | RLIB_CTYPE_SPACE); }
INLINE int rlib_ispunct(int c) { return _ctypes[c] & (RLIB_CTYPE_PUNCT); }
INLINE int rlib_isspace(int c) { return _ctypes[c] & (RLIB_CTYPE_SPACE); }
INLINE int rlib_isupper(int c) { return _ctypes[c] & (RLIB_CTYPE_UPPER); }
INLINE int rlib_isxdigit(int c) { return _ctypes[c] & (RLIB_CTYPE_XDIGIT); }
INLINE int rlib_tolower(int c) { return (c >= 'A' && c <= 'Z') ? c + 0x20 : c; }
INLINE int rlib_toupper(int c) { return (c >= 'a' && c <= 'z') ? c - 0x20 : c; }
INLINE int rlib_isascii(int c) { return (c >= 0 && c <= 0x7F); }

// #endif

#endif // R_LIB_H
