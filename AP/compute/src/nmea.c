/*
 * This file is part of the Nauteff Autopilot project.
 *
 * Copyright (C) 2022 Nauteff https://nauteff.com
 *
 * This library is free software: you can redistribute it and/or modify
 * it under the terms of the GNU Lesser General Public License as published by
 * the Free Software Foundation, either version 3 of the License, or
 * (at your option) any later version.
 *
 * This library is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU Lesser General Public License for more details.
 *
 * You should have received a copy of the GNU Lesser General Public License
 * along with this library.  If not, see <http://www.gnu.org/licenses/>.
 */

/*
 * @file Module "Nauteff_nmea"
 * Ce module assure le décodage de trames NMEA0183.
 *
 * Adapté au projet Nauteff il est conçu pour une implémentation
 * sur microcontrôleur. Il a une empreinte mémoire contrôlée,
 * il utilise peu la pile, et n'utilise pas d'allocation dynamique.
 * Son code est optimisé pour être de taille limitée et rapide.
 * Il n'utilise pas de librairie externe.
 * Il effectue des calculs sur entiers et nombres réels
 * simple ou double précision. Il permet de choisir
 * la précision des nombres réels et les trames décodées.
 *
 */

#include <stdint.h>
#include <string.h>
//#include <ctype.h>
//#include <stdio.h> /* for debugging : printf and other stuff */
#include "rlib.h"
#include "nmea.h"

int nmeasplit(const char *stc, const size_t len, const char *format, void **dataAdresses);
int nmea0183Decode_APB(const char *sentence, const int len, Nmea0183SentData *data);
int nmea0183Decode_RMB(const char *sentence, const int len, Nmea0183SentData *data);
int nmea0183Decode_RMC(const char *sentence, const int len, Nmea0183SentData *data);
int nmea0183Decode_MWV(const char *sentence, const int len, Nmea0183SentData *data);

typedef struct
{
    char *nameID;
    int type;
    int (*decodeFunction)(const char *sentence, const int len, Nmea0183SentData *data);
} SentenceTypeDesc;

static const SentenceTypeDesc sentenceTypeDesc[] = {
    {"APB", Nmea0183_APB, &nmea0183Decode_APB},
    {"RMB", Nmea0183_APB, &nmea0183Decode_RMB},
    {"RMC", Nmea0183_APB, &nmea0183Decode_RMC},
    {"MWV", Nmea0183_MWV, &nmea0183Decode_MWV}};

/* define number of sentenceTypeDesc */
#define nbSentenceTypeDesc (sizeof(sentenceTypeDesc) / sizeof(SentenceTypeDesc))


// WARNING : This function shall not be used with non hexadecimal characters
static unsigned char hexc2uc(const char c)
{
    if (c >= '0' && c <= '9')
    {
        return c - '0';
    }
    else
    {
        if (c >= 'A' && c <= 'F')
        {
            return c - 'A' + 10;
        }
        else
        {
            return c - 'a' + 10;
        }
    }
}

/**
 * @fn nmea0183Decode
 *
 * @brief decode a NMEA 0183 frame
 *
 * @param frame to decode
 * @param data pointer to Nmea0183SentTypeData structure
 *
 * frame must be a null terminated string.
 *
 * @return int : -1 on error, frame type on success
 */

int nmea0183Decode(const char *frame, Nmea0183SentTypeData *data)
{
    uint8_t idxEnd; /* Pointer to end of frame */
    uint8_t i;
    uint16_t numType;
    //char sentType[4];
    unsigned char sentcs, compcs; // sentence checksum and computed checksum
    uint16_t sz;

    /* Size of frame : classic loop on frame[] */
    for (sz = 0; frame[sz] != '\0'; sz++)
    {
        ;
    }

    /* Check if size is enough at least $AAAAA for origin and sentence type
       and *xx et end for checksum : at least 9 characters
       and not greater than 82 */
    if (sz < 9 || sz > 82)
    {
        return -1;
    }

    /* Set a cursor at end of frame */
    idxEnd = sz - 1; // idxEnd is the position of the last character in sentence

    /* Strip if necessary one or two last characters
       if they are <CR> or <LF>, no matter the order
       this is done twice */
    for (i = 0; i < 2; i++)
    {
        if (frame[idxEnd] == '\n' || frame[idxEnd] == '\r')
        {
            idxEnd--;
        }
    }

    /* IdxEnd points to
     char cccc = sentence[idxEnd]; */

    /* Only sentences beginning with a $ are analyzed
       Check the $ at the beginning of the sentence*/
    if (frame[0] != '$')
    {
        return -1;
    }

    /* The checksum is provided by two last characters after a *
       and before <CR><LF>
       Check *XX at last position before <CR> and <LF>
       XX is the hexadecimal value of checksum
       cccc = sentence[idxEnd-2]; */

    if (frame[idxEnd - 2] != '*')
    {
        return -1;
    }
    if (!rlib_isxdigit(frame[idxEnd - 1]) || !rlib_isxdigit(frame[idxEnd]))
    {
        return -1;
    }

    /* Compute the value of the checksum (that has been sent with the the frame)
       at the end of the sentence : */
    compcs = (hexc2uc(frame[idxEnd - 1]) << 4) + hexc2uc(frame[idxEnd]);
    idxEnd -= 3;
    /** idxEnd indicates the size of useful sentence aka payload
     * without checksum and line end 
     * It is the position of the * character */

    /* Compute checksumm of the sentence. */
    sentcs = 0;
    for (i = 1; i <= idxEnd; i++)
    {
        sentcs ^= frame[i];
    }

    /* Compare checksums and reject sentence if checksums are different */
    if (sentcs != compcs)
    {
        return -1;
    }

    /* Check : first character is a dollar sign, if not reject the frame */
    /* since this library decodes only frames beginning with $ */
    if (frame[0] != '$')
    {
        return -1;
    }

    /* Store the talker id (origin of the frame) in data->origin. */
    /* the origin is in the characters 1-2 of the frame */
    data->origin[0] = frame[1];
    data->origin[1] = frame[2];
    data->origin[2] = '\0';

    /* Store the type of frame in data->typeName. */
    /* the type is in the characters 3-5 of the frame */
    data->typeName[0] = frame[3];
    data->typeName[1] = frame[4];
    data->typeName[2] = frame[5];
    data->typeName[3] = '\0';

    // look for descriptor in sentenceTypeDesc
    // TODO : use a qsort() like search for better efficiency.
    numType = -1;
    for (i = 0; i < nbSentenceTypeDesc; i++)
    {
        if (strncmp(frame + 3, sentenceTypeDesc[i].nameID, (unsigned long int)3) == 0)
        {
            numType = sentenceTypeDesc[i].type;
            break;
        }
    }
    data->type = numType;

    if (numType == -1)
    {
        return -1;
    }

    if (frame[6] != ',')
    {
        return -1;
    }

    /* Appel de la fonction de découpage et de conversions          */
    /* the address of the function is in the field decodeFunction   */
    /* in the numtype record. the decode function precesses frame   */
    /* after $, talkerId (2chars) type(3 chars) and comma (7 chars) */
    /* Decode function put data in  data */
    int res = (*(sentenceTypeDesc[numType].decodeFunction))(frame + 7, idxEnd - 7, &(data->data));
    if (res >= 0)
    {
        data->type = numType;
        return numType;
    }
    else
    {
        return -1;
    }
}

#if 0
int nmeaisxdigit(const char c);
int nmeaisxdigit(const char c) // TODO : inline version
{
    return ((c >= '0' && c <= '9') || (c >= 'A' && c <= 'Z') || (c >= 'A' && c <= 'Z')) ? 1 : 0;
}
#endif

static unsigned char hexc2uc(const char c);

/*
 * F : Floting point Number float or double
 * I : Integer
 * S : String 
 * C : character
 * G : Longitude  dddmm.mmm... 3 digits for degrees an minutes
 * L : Lattitude  ddmm.mmm... 2 digits for degrees an minutes
 * 
 * M : 'M' or 'T' Magnetic or True
 * O : 'L' or 'R' Left or Right
 * R : 'R' or 'T' Relative or True
 * E : 'E' or 'W' East or West
 * 
 * T : Time hhmmss.ss fractionnal part optional
 */

int nmea0183Decode_APB(const char *sentence, const int len, Nmea0183SentData *data)
{
    return 0;
}

int nmea0183Decode_RMB(const char *sentence, const int len, Nmea0183SentData *data)
{
    return 0;
}

int nmea0183Decode_RMC(const char *sentence, const int len, Nmea0183SentData *data)
{
    const static char format[] = "TCLCGCFFDF";
    const void *dataAdresses[10];
    dataAdresses[0] = &data->rmc.time_ms;
    dataAdresses[1] = &data->rmc.latitude;
    

    //int res;
    (void)dataAdresses;

    return 0;
}

int nmea0183Decode_MWV(const char *sentence, const int len, Nmea0183SentData *data)
{
    const static char format[] = "DRDCV";
    const void *dataAdresses[5];
    int res;

    dataAdresses[0] = &data->mwv.windAngle;
    dataAdresses[1] = &data->mwv.windRef;
    dataAdresses[2] = &data->mwv.windSpeed;
    dataAdresses[3] = &data->mwv.speedUnit;
    dataAdresses[4] = &data->mwv.status;
    res = nmeasplit(sentence, len, format, (void **)dataAdresses);

    if (res == 5)
    {
        return Nmea0183_MWV;
    }
    else
    {
        return -1;
    }
}

/*
 * @brief This function decode a NMEA sentence and store the values.
 *
 * nmeasplit scans fields between fields and converts them.
 * The fields are separated by commas. The list of fields is in format, .
 * The type of
 *
 * nmeasplit() searches for fields in the string, converts them,
 * and stores the values in variables whose addresses are in data.
 * The fields are separated by commas. The types of variables
 * are specified in the format string, with one character per field.
 * The Letters of format and corresponding types are :
 *   A : one character A or V, A for valid, V for void
 *   C : one character
 *   D : Decimal, float
 *   S : string,
 *   M : one character M or T, M for magnetic, T for true
 *
 *
 * @param stc : sentence to be decoded
 * @param len : length of stc
 * @param format : the format of the sentence
 * @param data : the pointer to an array containing pointer of the data
 * @return -1 if an error occured, or the number of fields decoded
 */

int nmeasplit(const char *stc, const size_t len, const char *format, void **data)
{
    int nbConvertedFields = 0;
    int nbFields = 0;        // Number of fields
    unsigned int idxStc = 0; // Index : position in the string (part of data of NMEA sentence)
    int fieldEnd;
    int fieldErr;
    int i1, i2;
    // union {
    //	int   integer;
    //	float decimal;
    //	char  character;
    // }fieldValue;
    unsigned dpp; // Decimal point position for decimal numbers, latitude, longitude
    uint8_t commaPositions[NMEA_NB_MAX_FIELDS];
    unsigned int idxField;

    // First count the number of fields in the string
    // fields are separated by commas, the for loop counts the commas
    for (idxStc = 0; idxStc < len && nbFields < NMEA_NB_MAX_FIELDS; idxStc++)
    {
        if (stc[idxStc] == ',')
        {
            commaPositions[nbFields] = (uint8_t)idxStc;
            nbFields++;
        }
    }
    // Number of fields is number of commas + 1
    nbFields++;

    idxStc = 0;
    for (idxField = 0; idxField < nbFields; idxField++)
    {
        char c = format[idxField];
        switch (format[idxField])
        {

        case 'A': // Letter 'A' ou 'V'
            if (stc[idxStc] == 'A' || stc[idxStc] == 'V')
            {
                *((char *)data[idxField]) = stc[idxStc++];
            }
            else
            {
                return -1;
            }
            break;

        case 'C': // Character
            *((char *)data[idxField]) = stc[idxStc++];
            break;

        case 'D': // Decimal
            dpp = 0;
            fieldEnd = 0;
            fieldErr = 0;
            i1 = i2 = 0;
            // fieldValue.decimal = 0.F;
            do
            {
                char c = stc[idxStc++];
                if (nmeaisxdigit(c))
                {
                    if (!dpp)
                    {
                        i1 = i1 * 10 + (int)(c - '0');
                    }
                    else
                    {
                        i2 = i2 * 10 + (int)(c - '0');
                    }
                }
                else
                {
                    if (c == '.' && dpp == 0)
                    {
                        if (dpp == 0)
                        {
                            dpp = 1;
                        }
                        else
                        {
                            fieldErr = 1;
                            fieldEnd = 1;
                        }
                    }
                }
                if (idxStc == commaPositions[idxField])
                {
                    fieldEnd = 1;
                }
            } while (!fieldEnd);
            if (fieldErr)
            {
                return -1;
            }
            // if ((i1 == 0) & (i2==0)) {
            //	return -1;
            // }
            //  TODO : Add decimal part
            *((float *)(data[idxField])) = (float)i1;
            break;

        case 'I': // Integer
            break;

        case 'M': // 'M' or 'T' Magnetic or True
            if (stc[idxStc] == 'M' || stc[idxStc] == 'T')
            {
                *((char *)data[idxField]) = stc[idxStc++];
            }
            else
            {
                return -1;
            }
            break;

        case 'N': // Number integer or decimal
            break;

        case 'O': // 'L' or 'R' Left or Right
            if (stc[idxStc] == 'L' || stc[idxStc] == 'R')
            {
                *((char *)data[idxField]) = stc[idxStc++];
            }
            else
            {
                return -1;
            }
            break;

        case 'R': // 'R' or 'T' Relative or True
            if (stc[idxStc] == 'R' || stc[idxStc] == 'T')
            {
                *((char *)data[idxField]) = stc[idxStc++];
            }
            else
            {
                return -1;
            }
            break;

        case 'S': // String
            break;

        case 'U':
            break;

        case 'V': // 'A' or 'V' A or Valid
            if (stc[idxStc] == 'A' || stc[idxStc] == 'V')
            {
                *((char *)data[idxField]) = stc[idxStc++];
            }
            else
            {
                return -1;
            }
            break;

        default:
            return -1;
            break;
        }
        idxStc++;
    }
    // TODO check if all data have been read
    // When last field is decoded idxStc is beyond the position of the last character
    // it is at len + 1, then at the end of the loop idxStc is incremented to skip
    // a comma
    if (idxStc != len + 2)
    {
        return -1;
    }

    return idxField;
}
