#include <stddef.h>
#include <stdint.h>
#include <util.h>
#include <rlibconfig.h>
#include <rlib.h>

#if RLIB_USE_MEMSET
void *memset(void *s, int c, size_t n)
{
    unsigned char *p = s;
    while (n--)
    {
        *p++ = (unsigned char)c;
    }
    return s;
}
#endif

#if RLIB_USE_MEMCPY
void *memcpy(void *dest, const void *src, size_t n)
{
    unsigned char *d = dest;
    const unsigned char *s = src;
    while (n--)
    {
        *d++ = *s++;
    }
    return dest;
}
#endif

#if RLIB_USE_STRLEN
size_t strlen(const char *s)
{
    const char *p = s;
    while (*p)
    {
        p++;
    }
    return p - s;
}
#endif

#if RLIB_USE_STRCPY
char *strcpy(char *dst, const char *src)
{
    while (*dst++ == *src++)
    {
        ;
    }

    return dst;
}

#endif /* RLIB_USE_STRCPY */

#if RLIB_USE_CTYPE
char _ctypes[256] = {
    /* TODO : mettre au point */
    RLIB_CTYPE_CNTRL,                     /* 0x00  NUL « \0 » (octet NULL) */
    RLIB_CTYPE_CNTRL,                     /* 0x01  SOH (début d'en-tête) */
    RLIB_CTYPE_CNTRL,                     /* 0x02  STX (début de texte) */
    RLIB_CTYPE_CNTRL,                     /* 0x03 ETX (fin de texte) */
    RLIB_CTYPE_CNTRL,                     /* 0x04 EOT (fin de transmission) */
    RLIB_CTYPE_CNTRL,                     /* 0x05 ENQ (requête) */
    RLIB_CTYPE_CNTRL,                     /* 0x06 ACK (accusé de réception) */
    RLIB_CTYPE_CNTRL,                     /* 0x07 BEL « \a » (sonnerie) */
    RLIB_CTYPE_CNTRL,                     /* 0x08 BS  « \b » (espace arrière) */
    RLIB_CTYPE_CNTRL | RLIB_CTYPE_SPACE,  /* 0x09 HT  « \t » (tabulation horizontale) */
    RLIB_CTYPE_CNTRL | RLIB_CTYPE_SPACE,  /* 0x0A LF  « \n » (saut de ligne) */
    RLIB_CTYPE_CNTRL | RLIB_CTYPE_SPACE,  /* 0x0B VT  « \v » (tabulation verticale) */
    RLIB_CTYPE_CNTRL | RLIB_CTYPE_SPACE,  /* 0x0C FF  « \f » (saut de page) */
    RLIB_CTYPE_CNTRL | RLIB_CTYPE_SPACE,  /* 0x0D CR  « \r » (retour chariot) */
    RLIB_CTYPE_CNTRL,                     /* 0x0E SO  (code spécial) */
    RLIB_CTYPE_CNTRL,                     /* 0x0F SI  (code standard) */
    RLIB_CTYPE_CNTRL,                     /* 0x10 DLE (échappement en transmission) */
    RLIB_CTYPE_CNTRL,                     /* 0x11 DC1 (contrôle de périphérique 1) */
    RLIB_CTYPE_CNTRL,                     /* 0x12 DC2 (contrôle de périphérique 2) */
    RLIB_CTYPE_CNTRL,                     /* 0x13 DC3 (contrôle de périphérique 3) */
    RLIB_CTYPE_CNTRL,                     /* 0x14 DC4 (contrôle de périphérique 4) */
    RLIB_CTYPE_CNTRL,                     /* 0x15 NAK (accusé de réception nég.) */
    RLIB_CTYPE_CNTRL,                     /* 0x16 SYN (attente synchronisée) */
    RLIB_CTYPE_CNTRL,                     /* 0x17 ETB (fin de bloc de transmission) */
    RLIB_CTYPE_CNTRL,                     /* 0x18 CAN (annulation) */
    RLIB_CTYPE_CNTRL,                     /* 0x19 EM  (fin de support) */
    RLIB_CTYPE_CNTRL,                     /* 0x1A SUB (substitution) */
    RLIB_CTYPE_CNTRL,                     /* 0x1B  ESC (échappement) */
    RLIB_CTYPE_CNTRL,                     /* 0x1C  FS  (séparateur de fichier) */
    RLIB_CTYPE_CNTRL,                     /* 0x1D  GS  (séparateur de groupe) */
    RLIB_CTYPE_CNTRL,                     /* 0x1E  RS  (séparateur d'enregistrement) */
    RLIB_CTYPE_CNTRL,                     /* 0x1F  US  (séparateur d’unité) */
    RLIB_CTYPE_SPACE,                     /* 0x20 : SPACE */
    RLIB_CTYPE_PUNCT,                     /* 0x21 : ! */
    RLIB_CTYPE_PUNCT,                     /* 0x22 : " */
    RLIB_CTYPE_PUNCT,                     /* 0x23 : # */
    RLIB_CTYPE_PUNCT,                     /* 0x24 : $ */
    RLIB_CTYPE_PUNCT,                     /* 0x25 : % */
    RLIB_CTYPE_PUNCT,                     /* 0x26 : & */
    RLIB_CTYPE_PUNCT,                     /* 0x27 : ' */
    RLIB_CTYPE_PUNCT,                     /* 0x28 : ( */
    RLIB_CTYPE_PUNCT,                     /* 0x29 : ) */
    RLIB_CTYPE_PUNCT,                     /* 0x2A : * */
    RLIB_CTYPE_PUNCT,                     /* 0x2B : + */
    RLIB_CTYPE_PUNCT,                     /* 0x2C : , */
    RLIB_CTYPE_PUNCT,                     /* 0x2D : - */
    RLIB_CTYPE_PUNCT,                     /* 0x2E : . */
    RLIB_CTYPE_PUNCT,                     /* 0x2F : / */
    RLIB_CTYPE_DIGIT,                     /* 0x30 : 0 */
    RLIB_CTYPE_DIGIT,                     /* 0x31 : 1 */
    RLIB_CTYPE_DIGIT,                     /* 0x32 : 2 */
    RLIB_CTYPE_DIGIT,                     /* 0x33 : 3 */
    RLIB_CTYPE_DIGIT,                     /* 0x34 : 4 */
    RLIB_CTYPE_DIGIT,                     /* 0x35 : 5 */
    RLIB_CTYPE_DIGIT,                     /* 0x36 : 6 */
    RLIB_CTYPE_DIGIT,                     /* 0x37 : 7 */
    RLIB_CTYPE_DIGIT,                     /* 0x38 : 8 */
    RLIB_CTYPE_DIGIT,                     /* 0x39 : 9 */
    RLIB_CTYPE_PUNCT,                     /* 0x3A : : */
    RLIB_CTYPE_PUNCT,                     /* 0x3B : ; */
    RLIB_CTYPE_PUNCT,                     /* 0x3C : < */
    RLIB_CTYPE_PUNCT,                     /* 0x3D : = */
    RLIB_CTYPE_PUNCT,                     /* 0x3E : > */
    RLIB_CTYPE_PUNCT,                     /* 0x3F : ? */
    RLIB_CTYPE_PUNCT,                     /* 0x40 : @ */
    RLIB_CTYPE_UPPER | RLIB_CTYPE_XDIGIT, /* 0x41 : A */
    RLIB_CTYPE_UPPER | RLIB_CTYPE_XDIGIT, /* 0x42 : B */
    RLIB_CTYPE_UPPER | RLIB_CTYPE_XDIGIT, /* 0x43 : C */
    RLIB_CTYPE_UPPER | RLIB_CTYPE_XDIGIT, /* 0x44 : D */
    RLIB_CTYPE_UPPER | RLIB_CTYPE_XDIGIT, /* 0x45 : E */
    RLIB_CTYPE_UPPER | RLIB_CTYPE_XDIGIT, /* 0x46 : F */
    RLIB_CTYPE_UPPER,                     /* 0x47 : G */
    RLIB_CTYPE_UPPER,                     /* 0x48 : H */
    RLIB_CTYPE_UPPER,                     /* 0x49 : I */
    RLIB_CTYPE_UPPER,                     /* 0x4A : J */
    RLIB_CTYPE_UPPER,                     /* 0x4B : K */
    RLIB_CTYPE_UPPER,                     /* 0x4C : L */
    RLIB_CTYPE_UPPER,                     /* 0x4D : M */
    RLIB_CTYPE_UPPER,                     /* 0x4E : N */
    RLIB_CTYPE_UPPER,                     /* 0x4F : O */
    RLIB_CTYPE_UPPER,                     /* 0x50 : P */
    RLIB_CTYPE_UPPER,                     /* 0x51 : Q */
    RLIB_CTYPE_UPPER,                     /* 0x52 : R */
    RLIB_CTYPE_UPPER,                     /* 0x53 : S */
    RLIB_CTYPE_UPPER,                     /* 0x54 : T */
    RLIB_CTYPE_UPPER,                     /* 0x55 : U */
    RLIB_CTYPE_UPPER,                     /* 0x56 : V */
    RLIB_CTYPE_UPPER,                     /* 0x57 : W */
    RLIB_CTYPE_UPPER,                     /* 0x58 : X */
    RLIB_CTYPE_UPPER,                     /* 0x59 : Y */
    RLIB_CTYPE_UPPER,                     /* 0x5A : Z */
    RLIB_CTYPE_PUNCT,                     /* 0x5B : [ */
    RLIB_CTYPE_PUNCT,                     /* 0x5C : \ */
    RLIB_CTYPE_PUNCT,                     /* 0x5D : ] */
    RLIB_CTYPE_PUNCT,                     /* 0x5E : ^ */
    RLIB_CTYPE_PUNCT,                     /* 0x5F : _ */
    RLIB_CTYPE_PUNCT,                     /* 0x60 : ` */
    RLIB_CTYPE_LOWER | RLIB_CTYPE_XDIGIT, /* 0x61 : a */
    RLIB_CTYPE_LOWER | RLIB_CTYPE_XDIGIT, /* 0x62 : b */
    RLIB_CTYPE_LOWER | RLIB_CTYPE_XDIGIT, /* 0x63 : c */
    RLIB_CTYPE_LOWER | RLIB_CTYPE_XDIGIT, /* 0x64 : d */
    RLIB_CTYPE_LOWER | RLIB_CTYPE_XDIGIT, /* 0x65 : e */
    RLIB_CTYPE_LOWER | RLIB_CTYPE_XDIGIT, /* 0x66 : f */
    RLIB_CTYPE_LOWER,                     /* 0x67 : g */
    RLIB_CTYPE_LOWER,                     /* 0x68 : h */
    RLIB_CTYPE_LOWER,                     /* 0x69 : i */
    RLIB_CTYPE_LOWER,                     /* 0x6A : j */
    RLIB_CTYPE_LOWER,                     /* 0x6B : k */
    RLIB_CTYPE_LOWER,                     /* 0x6C : l */
    RLIB_CTYPE_LOWER,                     /* 0x6D : m */
    RLIB_CTYPE_LOWER,                     /* 0x6E : n */
    RLIB_CTYPE_LOWER,                     /* 0x6F : o */
    RLIB_CTYPE_LOWER,                     /* 0x70 : p */
    RLIB_CTYPE_LOWER,                     /* 0x71 : q */
    RLIB_CTYPE_LOWER,                     /* 0x72 : r */
    RLIB_CTYPE_LOWER,                     /* 0x73 : s */
    RLIB_CTYPE_LOWER,                     /* 0x74 : t */
    RLIB_CTYPE_LOWER,                     /* 0x75 : u */
    RLIB_CTYPE_LOWER,                     /* 0x76 : v */
    RLIB_CTYPE_LOWER,                     /* 0x77 : w */
    RLIB_CTYPE_LOWER,                     /* 0x78 : x */
    RLIB_CTYPE_LOWER,                     /* 0x79 : y */
    RLIB_CTYPE_LOWER,                     /* 0x7A : z */
    RLIB_CTYPE_PUNCT,                     /* 0x7B : { */
    RLIB_CTYPE_PUNCT,                     /* 0x7C : | */
    RLIB_CTYPE_PUNCT,                     /* 0x7D : } */
    RLIB_CTYPE_PUNCT,                     /* 0x7E : ~ */
    RLIB_CTYPE_CNTRL,                     /* 0x7F : DEL */
    0,                                    /* 0x80 */
    0,                                    /* 0x81 */
    0,                                    /* 0x82 */
    0,                                    /* 0x83 */
    0,                                    /* 0x84 */
    0,                                    /* 0x85 */
    0,                                    /* 0x86 */
    0,                                    /* 0x87 */
    0,                                    /* 0x88 */
    0,                                    /* 0x89 */
    0,                                    /* 0x8A */
    0,                                    /* 0x8B */
    0,                                    /* 0x8C */
    0,                                    /* 0x8D */
    0,                                    /* 0x8E */
    0,                                    /* 0x8F */
    0,                                    /* 0x90 */
    0,                                    /* 0x91 */
    0,                                    /* 0x92 */
    0,                                    /* 0x93 */
    0,                                    /* 0x94 */
    0,                                    /* 0x95 */
    0,                                    /* 0x96 */
    0,                                    /* 0x97 */
    0,                                    /* 0x98 */
    0,                                    /* 0x99 */
    0,                                    /* 0x9A */
    0,                                    /* 0x9B */
    0,                                    /* 0x9C */
    0,                                    /* 0x9D */
    0,                                    /* 0x9E */
    0,                                    /* 0x9F */
    0,                                    /* 0xA0 */
    0,                                    /* 0xA1 */
    0,                                    /* 0xA2 */
    0,                                    /* 0xA3 */
    0,                                    /* 0xA4 */
    0,                                    /* 0xA5 */
    0,                                    /* 0xA6 */
    0,                                    /* 0xA7 */
    0,                                    /* 0xA8 */
    0,                                    /* 0xA9 */
    0,                                    /* 0xAA */
    0,                                    /* 0xAB */
    0,                                    /* 0xAC */
    0,                                    /* 0xAD */
    0,                                    /* 0xAE */
    0,                                    /* 0xAF */
    0,                                    /* 0xB0 */
    0,                                    /* 0xB1 */
    0,                                    /* 0xB2 */
    0,                                    /* 0xB3 */
    0,                                    /* 0xB4 */
    0,                                    /* 0xB5 */
    0,                                    /* 0xB6 */
    0,                                    /* 0xB7 */
    0,                                    /* 0xB8 */
    0,                                    /* 0xB9 */
    0,                                    /* 0xBA */
    0,                                    /* 0xBB */
    0,                                    /* 0xBC */
    0,                                    /* 0xBD */
    0,                                    /* 0xBE */
    0,                                    /* 0xBF */
    0,                                    /* 0xC0 */
    0,                                    /* 0xC1 */
    0,                                    /* 0xC2 */
    0,                                    /* 0xC3 */
    0,                                    /* 0xC4 */
    0,                                    /* 0xC5 */
    0,                                    /* 0xC6 */
    0,                                    /* 0xC7 */
    0,                                    /* 0xC8 */
    0,                                    /* 0xC9 */
    0,                                    /* 0xCA */
    0,                                    /* 0xCB */
    0,                                    /* 0xCC */
    0,                                    /* 0xCD */
    0,                                    /* 0xCE */
    0,                                    /* 0xCF */
    0,                                    /* 0xD0 */
    0,                                    /* 0xD1 */
    0,                                    /* 0xD2 */
    0,                                    /* 0xD3 */
    0,                                    /* 0xD4 */
    0,                                    /* 0xD5 */
    0,                                    /* 0xD6 */
    0,                                    /* 0xD7 */
    0,                                    /* 0xD8 */
    0,                                    /* 0xD9 */
    0,                                    /* 0xDA */
    0,                                    /* 0xDB */
    0,                                    /* 0xDC */
    0,                                    /* 0xDD */
    0,                                    /* 0xDE */
    0,                                    /* 0xDF */
    0,                                    /* 0xE0 */
    0,                                    /* 0xE1 */
    0,                                    /* 0xE2 */
    0,                                    /* 0xE3 */
    0,                                    /* 0xE4 */
    0,                                    /* 0xE5 */
    0,                                    /* 0xE6 */
    0,                                    /* 0xE7 */
    0,                                    /* 0xE8 */
    0,                                    /* 0xE9 */
    0,                                    /* 0xEA */
    0,                                    /* 0xEB */
    0,                                    /* 0xEC */
    0,                                    /* 0xED */
    0,                                    /* 0xEE */
    0,                                    /* 0xEF */
    0,                                    /* 0xF0 */
    0,                                    /* 0xF1 */
    0,                                    /* 0xF2 */
    0,                                    /* 0xF3 */
    0,                                    /* 0xF4 */
    0,                                    /* 0xF5 */
    0,                                    /* 0xF6 */
    0,                                    /* 0xF7 */
    0,                                    /* 0xF8 */
    0,                                    /* 0xF9 */
    0,                                    /* 0xFA */
    0,                                    /* 0xFB */
    0,                                    /* 0xFC */
    0,                                    /* 0xFD */
    0,                                    /* 0xFE */
    0                                     /* 0xFF */
};

unsigned char ctype_table[256] = {
    [0x00 ... 0x1F] = RLIB_CTYPE_CNTRL,
    [0x20] = RLIB_CTYPE_SPACE | RLIB_CTYPE_SPACE,
    [0x21 ... 0x2F] = RLIB_CTYPE_PUNCT,
    [0x30 ... 0x39] = RLIB_CTYPE_DIGIT | RLIB_CTYPE_XDIGIT,
    [0x3A ... 0x40] = RLIB_CTYPE_PUNCT,
    [0x41 ... 0x46] = RLIB_CTYPE_UPPER | RLIB_CTYPE_XDIGIT,
    [0x47 ... 0x5A] = RLIB_CTYPE_UPPER,
    [0x5B ... 0x60] = RLIB_CTYPE_PUNCT,
    [0x61 ... 0x66] = RLIB_CTYPE_LOWER | RLIB_CTYPE_XDIGIT,
    [0x67 ... 0x7A] = RLIB_CTYPE_LOWER,
    [0x7B ... 0x7E] = RLIB_CTYPE_PUNCT,
    [0x7F] = RLIB_CTYPE_CNTRL,
    [0x80 ... 0xFF] = 0 /* Caractères étendus, non classifiés */
};

#endif /* RLIB_USE_CTYPE */