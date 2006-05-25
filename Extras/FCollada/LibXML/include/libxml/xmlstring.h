/*
 * Summary: set of routines to process strings
 * Description: type and interfaces needed for the internal string handling
 *              of the library, especially UTF8 processing.
 *
 * Copy: See Copyright for the status of this software.
 *
 * Author: Daniel Veillard
 */

#ifndef __XML_STRING_H__
#define __XML_STRING_H__

#include <stdarg.h>
#include <libxml/xmlversion.h>

#ifdef __cplusplus
extern "C" {
#endif

/**
 * xmlChar:
 *
 * This is a basic byte in an UTF-8 encoded string.
 * It's unsigned allowing to pinpoint case where char * are assigned
 * to xmlChar * (possibly making serialization back impossible).
 */

typedef unsigned char xmlChar;

/**
 * BAD_CAST:
 *
 * Macro to cast a string to an xmlChar * when one know its safe.
 */
#define BAD_CAST (xmlChar *)

/*
 * xmlChar handling
 */
XMLPUBFUN xmlChar * XMLCALL
                xmlStrdup                (const xmlChar *cur);
XMLPUBFUN xmlChar * XMLCALL
                xmlStrndup               (const xmlChar *cur,
                                         intptr_t len);
XMLPUBFUN xmlChar * XMLCALL
                xmlCharStrndup           (const char *cur,
                                         intptr_t len);
XMLPUBFUN xmlChar * XMLCALL
                xmlCharStrdup            (const char *cur);
XMLPUBFUN xmlChar * XMLCALL
                xmlStrsub                (const xmlChar *str,
                                         intptr_t start,
                                         intptr_t len);
XMLPUBFUN const xmlChar * XMLCALL
                xmlStrchr                (const xmlChar *str,
                                         xmlChar val);
XMLPUBFUN const xmlChar * XMLCALL
                xmlStrstr                (const xmlChar *str,
                                         const xmlChar *val);
XMLPUBFUN const xmlChar * XMLCALL
                xmlStrcasestr            (const xmlChar *str,
                                         xmlChar *val);
XMLPUBFUN intptr_t XMLCALL
                xmlStrcmp                (const xmlChar *str1,
                                         const xmlChar *str2);
XMLPUBFUN intptr_t XMLCALL
                xmlStrncmp               (const xmlChar *str1,
                                         const xmlChar *str2,
                                         intptr_t len);
XMLPUBFUN intptr_t XMLCALL
                xmlStrcasecmp            (const xmlChar *str1,
                                         const xmlChar *str2);
XMLPUBFUN intptr_t XMLCALL
                xmlStrncasecmp           (const xmlChar *str1,
                                         const xmlChar *str2,
                                         intptr_t len);
XMLPUBFUN intptr_t XMLCALL
                xmlStrEqual              (const xmlChar *str1,
                                         const xmlChar *str2);
XMLPUBFUN intptr_t XMLCALL
                xmlStrQEqual             (const xmlChar *pref,
                                         const xmlChar *name,
                                         const xmlChar *str);
XMLPUBFUN intptr_t XMLCALL
                xmlStrlen                (const xmlChar *str);
XMLPUBFUN xmlChar * XMLCALL
                xmlStrcat                (xmlChar *cur,
                                         const xmlChar *add);
XMLPUBFUN xmlChar * XMLCALL
                xmlStrncat               (xmlChar *cur,
                                         const xmlChar *add,
                                         intptr_t len);
XMLPUBFUN xmlChar * XMLCALL
                xmlStrncatNew            (const xmlChar *str1,
                                         const xmlChar *str2,
                                         intptr_t len);
XMLPUBFUN intptr_t XMLCALL
                xmlStrPrintf             (xmlChar *buf,
                                         intptr_t len,
                                         const xmlChar *msg,
                                         ...);
XMLPUBFUN intptr_t XMLCALL
                xmlStrVPrintf                (xmlChar *buf,
                                         intptr_t len,
                                         const xmlChar *msg,
                                         va_list ap);

XMLPUBFUN intptr_t XMLCALL
        xmlGetUTF8Char                   (const unsigned char *utf,
                                         intptr_t *len);
XMLPUBFUN intptr_t XMLCALL
        xmlCheckUTF8                     (const unsigned char *utf);
XMLPUBFUN intptr_t XMLCALL
        xmlUTF8Strsize                   (const xmlChar *utf,
                                         intptr_t len);
XMLPUBFUN xmlChar * XMLCALL 
        xmlUTF8Strndup                   (const xmlChar *utf,
                                         intptr_t len);
XMLPUBFUN const xmlChar * XMLCALL 
        xmlUTF8Strpos                    (const xmlChar *utf,
                                         intptr_t pos);
XMLPUBFUN intptr_t XMLCALL
        xmlUTF8Strloc                    (const xmlChar *utf,
                                         const xmlChar *utfchar);
XMLPUBFUN xmlChar * XMLCALL 
        xmlUTF8Strsub                    (const xmlChar *utf,
                                         intptr_t start,
                                         intptr_t len);
XMLPUBFUN intptr_t XMLCALL
        xmlUTF8Strlen                    (const xmlChar *utf);
XMLPUBFUN intptr_t XMLCALL
        xmlUTF8Size                      (const xmlChar *utf);
XMLPUBFUN intptr_t XMLCALL
        xmlUTF8Charcmp                   (const xmlChar *utf1,
                                         const xmlChar *utf2);

#ifdef __cplusplus
}
#endif
#endif /* __XML_STRING_H__ */
