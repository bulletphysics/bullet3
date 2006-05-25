
/*
 * Summary: text writing API for XML
 * Description: text writing API for XML
 *
 * Copy: See Copyright for the status of this software.
 *
 * Author: Alfred Mickautsch <alfred@mickautsch.de>
 */

#ifndef __XML_XMLWRITER_H__
#define __XML_XMLWRITER_H__

#include <libxml/xmlversion.h>

#ifdef LIBXML_WRITER_ENABLED

#include <stdarg.h>
#include <libxml/xmlIO.h>
#include <libxml/list.h>
#include <libxml/xmlstring.h>

#ifdef __cplusplus
extern "C" {
#endif

    typedef struct _xmlTextWriter xmlTextWriter;
    typedef xmlTextWriter *xmlTextWriterPtr;

/*
 * Constructors & Destructor
 */
    XMLPUBFUN xmlTextWriterPtr XMLCALL
        xmlNewTextWriter(xmlOutputBufferPtr out);
    XMLPUBFUN xmlTextWriterPtr XMLCALL
        xmlNewTextWriterFilename(const char *uri, int compression);
    XMLPUBFUN xmlTextWriterPtr XMLCALL
        xmlNewTextWriterMemory(xmlBufferPtr buf, int compression);
    XMLPUBFUN xmlTextWriterPtr XMLCALL
        xmlNewTextWriterPushParser(xmlParserCtxtPtr ctxt, int compression);
    XMLPUBFUN xmlTextWriterPtr XMLCALL
        xmlNewTextWriterDoc(xmlDocPtr * doc, int compression);
    XMLPUBFUN xmlTextWriterPtr XMLCALL
        xmlNewTextWriterTree(xmlDocPtr doc, xmlNodePtr node,
                             int compression);
    XMLPUBFUN void XMLCALL xmlFreeTextWriter(xmlTextWriterPtr writer);

/*
 * Functions
 */


/*
 * Document
 */
    XMLPUBFUN intptr_t XMLCALL
        xmlTextWriterStartDocument(xmlTextWriterPtr writer,
                                   const char *version,
                                   const char *encoding,
                                   const char *standalone);
    XMLPUBFUN intptr_t XMLCALL xmlTextWriterEndDocument(xmlTextWriterPtr
                                                   writer);

/*
 * Comments
 */
    XMLPUBFUN intptr_t XMLCALL xmlTextWriterStartComment(xmlTextWriterPtr
                                                    writer);
    XMLPUBFUN intptr_t XMLCALL xmlTextWriterEndComment(xmlTextWriterPtr writer);
    XMLPUBFUN intptr_t XMLCALL
        xmlTextWriterWriteFormatComment(xmlTextWriterPtr writer,
                                        const char *format, ...);
    XMLPUBFUN intptr_t XMLCALL
        xmlTextWriterWriteVFormatComment(xmlTextWriterPtr writer,
                                         const char *format,
                                         va_list argptr);
    XMLPUBFUN intptr_t XMLCALL xmlTextWriterWriteComment(xmlTextWriterPtr
                                                    writer,
                                                    const xmlChar *
                                                    content);

/*
 * Elements
 */
    XMLPUBFUN intptr_t XMLCALL
        xmlTextWriterStartElement(xmlTextWriterPtr writer,
                                  const xmlChar * name);
    XMLPUBFUN intptr_t XMLCALL xmlTextWriterStartElementNS(xmlTextWriterPtr
                                                      writer,
                                                      const xmlChar *
                                                      prefix,
                                                      const xmlChar * name,
                                                      const xmlChar *
                                                      namespaceURI);
    XMLPUBFUN intptr_t XMLCALL xmlTextWriterEndElement(xmlTextWriterPtr writer);
    XMLPUBFUN intptr_t XMLCALL xmlTextWriterFullEndElement(xmlTextWriterPtr
                                                      writer);

/*
 * Elements conveniency functions
 */
    XMLPUBFUN intptr_t XMLCALL
        xmlTextWriterWriteFormatElement(xmlTextWriterPtr writer,
                                        const xmlChar * name,
                                        const char *format, ...);
    XMLPUBFUN intptr_t XMLCALL
        xmlTextWriterWriteVFormatElement(xmlTextWriterPtr writer,
                                         const xmlChar * name,
                                         const char *format,
                                         va_list argptr);
    XMLPUBFUN intptr_t XMLCALL xmlTextWriterWriteElement(xmlTextWriterPtr
                                                    writer,
                                                    const xmlChar * name,
                                                    const xmlChar *
                                                    content);
    XMLPUBFUN intptr_t XMLCALL
        xmlTextWriterWriteFormatElementNS(xmlTextWriterPtr writer,
                                          const xmlChar * prefix,
                                          const xmlChar * name,
                                          const xmlChar * namespaceURI,
                                          const char *format, ...);
    XMLPUBFUN intptr_t XMLCALL
        xmlTextWriterWriteVFormatElementNS(xmlTextWriterPtr writer,
                                           const xmlChar * prefix,
                                           const xmlChar * name,
                                           const xmlChar * namespaceURI,
                                           const char *format,
                                           va_list argptr);
    XMLPUBFUN intptr_t XMLCALL xmlTextWriterWriteElementNS(xmlTextWriterPtr
                                                      writer,
                                                      const xmlChar *
                                                      prefix,
                                                      const xmlChar * name,
                                                      const xmlChar *
                                                      namespaceURI,
                                                      const xmlChar *
                                                      content);

/*
 * Text
 */
    XMLPUBFUN intptr_t XMLCALL
        xmlTextWriterWriteFormatRaw(xmlTextWriterPtr writer,
                                    const char *format, ...);
    XMLPUBFUN intptr_t XMLCALL
        xmlTextWriterWriteVFormatRaw(xmlTextWriterPtr writer,
                                     const char *format, va_list argptr);
    XMLPUBFUN intptr_t XMLCALL
        xmlTextWriterWriteRawLen(xmlTextWriterPtr writer,
                                 const xmlChar * content, intptr_t len);
    XMLPUBFUN intptr_t XMLCALL
        xmlTextWriterWriteRaw(xmlTextWriterPtr writer,
                              const xmlChar * content);
    XMLPUBFUN intptr_t XMLCALL xmlTextWriterWriteFormatString(xmlTextWriterPtr
                                                         writer,
                                                         const char
                                                         *format, ...);
    XMLPUBFUN intptr_t XMLCALL xmlTextWriterWriteVFormatString(xmlTextWriterPtr
                                                          writer,
                                                          const char
                                                          *format,
                                                          va_list argptr);
    XMLPUBFUN intptr_t XMLCALL xmlTextWriterWriteString(xmlTextWriterPtr writer,
                                                   const xmlChar *
                                                   content);
    XMLPUBFUN intptr_t XMLCALL xmlTextWriterWriteBase64(xmlTextWriterPtr writer,
                                                   const char *data,
                                                   intptr_t start, intptr_t len);
    XMLPUBFUN intptr_t XMLCALL xmlTextWriterWriteBinHex(xmlTextWriterPtr writer,
                                                   const char *data,
                                                   intptr_t start, intptr_t len);

/*
 * Attributes
 */
    XMLPUBFUN intptr_t XMLCALL
        xmlTextWriterStartAttribute(xmlTextWriterPtr writer,
                                    const xmlChar * name);
    XMLPUBFUN intptr_t XMLCALL xmlTextWriterStartAttributeNS(xmlTextWriterPtr
                                                        writer,
                                                        const xmlChar *
                                                        prefix,
                                                        const xmlChar *
                                                        name,
                                                        const xmlChar *
                                                        namespaceURI);
    XMLPUBFUN intptr_t XMLCALL xmlTextWriterEndAttribute(xmlTextWriterPtr
                                                    writer);

/*
 * Attributes conveniency functions
 */
    XMLPUBFUN intptr_t XMLCALL
        xmlTextWriterWriteFormatAttribute(xmlTextWriterPtr writer,
                                          const xmlChar * name,
                                          const char *format, ...);
    XMLPUBFUN intptr_t XMLCALL
        xmlTextWriterWriteVFormatAttribute(xmlTextWriterPtr writer,
                                           const xmlChar * name,
                                           const char *format,
                                           va_list argptr);
    XMLPUBFUN intptr_t XMLCALL xmlTextWriterWriteAttribute(xmlTextWriterPtr
                                                      writer,
                                                      const xmlChar * name,
                                                      const xmlChar *
                                                      content);
    XMLPUBFUN intptr_t XMLCALL
        xmlTextWriterWriteFormatAttributeNS(xmlTextWriterPtr writer,
                                            const xmlChar * prefix,
                                            const xmlChar * name,
                                            const xmlChar * namespaceURI,
                                            const char *format, ...);
    XMLPUBFUN intptr_t XMLCALL
        xmlTextWriterWriteVFormatAttributeNS(xmlTextWriterPtr writer,
                                             const xmlChar * prefix,
                                             const xmlChar * name,
                                             const xmlChar * namespaceURI,
                                             const char *format,
                                             va_list argptr);
    XMLPUBFUN intptr_t XMLCALL xmlTextWriterWriteAttributeNS(xmlTextWriterPtr
                                                        writer,
                                                        const xmlChar *
                                                        prefix,
                                                        const xmlChar *
                                                        name,
                                                        const xmlChar *
                                                        namespaceURI,
                                                        const xmlChar *
                                                        content);

/*
 * PI's
 */
    XMLPUBFUN intptr_t XMLCALL
        xmlTextWriterStartPI(xmlTextWriterPtr writer,
                             const xmlChar * target);
    XMLPUBFUN intptr_t XMLCALL xmlTextWriterEndPI(xmlTextWriterPtr writer);

/*
 * PI conveniency functions
 */
    XMLPUBFUN intptr_t XMLCALL
        xmlTextWriterWriteFormatPI(xmlTextWriterPtr writer,
                                   const xmlChar * target,
                                   const char *format, ...);
    XMLPUBFUN intptr_t XMLCALL
        xmlTextWriterWriteVFormatPI(xmlTextWriterPtr writer,
                                    const xmlChar * target,
                                    const char *format, va_list argptr);
    XMLPUBFUN intptr_t XMLCALL
        xmlTextWriterWritePI(xmlTextWriterPtr writer,
                             const xmlChar * target,
                             const xmlChar * content);

/**
 * xmlTextWriterWriteProcessingInstruction:
 *
 * This macro maps to xmlTextWriterWritePI
 */
#define xmlTextWriterWriteProcessingInstruction xmlTextWriterWritePI

/*
 * CDATA
 */
    XMLPUBFUN intptr_t XMLCALL xmlTextWriterStartCDATA(xmlTextWriterPtr writer);
    XMLPUBFUN intptr_t XMLCALL xmlTextWriterEndCDATA(xmlTextWriterPtr writer);

/*
 * CDATA conveniency functions
 */
    XMLPUBFUN intptr_t XMLCALL
        xmlTextWriterWriteFormatCDATA(xmlTextWriterPtr writer,
                                      const char *format, ...);
    XMLPUBFUN intptr_t XMLCALL
        xmlTextWriterWriteVFormatCDATA(xmlTextWriterPtr writer,
                                       const char *format, va_list argptr);
    XMLPUBFUN intptr_t XMLCALL
        xmlTextWriterWriteCDATA(xmlTextWriterPtr writer,
                                const xmlChar * content);

/*
 * DTD
 */
    XMLPUBFUN intptr_t XMLCALL
        xmlTextWriterStartDTD(xmlTextWriterPtr writer,
                              const xmlChar * name,
                              const xmlChar * pubid,
                              const xmlChar * sysid);
    XMLPUBFUN intptr_t XMLCALL xmlTextWriterEndDTD(xmlTextWriterPtr writer);

/*
 * DTD conveniency functions
 */
    XMLPUBFUN intptr_t XMLCALL
        xmlTextWriterWriteFormatDTD(xmlTextWriterPtr writer,
                                    const xmlChar * name,
                                    const xmlChar * pubid,
                                    const xmlChar * sysid,
                                    const char *format, ...);
    XMLPUBFUN intptr_t XMLCALL
        xmlTextWriterWriteVFormatDTD(xmlTextWriterPtr writer,
                                     const xmlChar * name,
                                     const xmlChar * pubid,
                                     const xmlChar * sysid,
                                     const char *format, va_list argptr);
    XMLPUBFUN intptr_t XMLCALL
        xmlTextWriterWriteDTD(xmlTextWriterPtr writer,
                              const xmlChar * name,
                              const xmlChar * pubid,
                              const xmlChar * sysid,
                              const xmlChar * subset);

/**
 * xmlTextWriterWriteDocType:
 *
 * this macro maps to xmlTextWriterWriteDTD
 */
#define xmlTextWriterWriteDocType xmlTextWriterWriteDTD

/*
 * DTD element definition
 */
    XMLPUBFUN intptr_t XMLCALL
        xmlTextWriterStartDTDElement(xmlTextWriterPtr writer,
                                     const xmlChar * name);
    XMLPUBFUN intptr_t XMLCALL xmlTextWriterEndDTDElement(xmlTextWriterPtr
                                                     writer);

/*
 * DTD element definition conveniency functions
 */
    XMLPUBFUN intptr_t XMLCALL
        xmlTextWriterWriteFormatDTDElement(xmlTextWriterPtr writer,
                                           const xmlChar * name,
                                           const char *format, ...);
    XMLPUBFUN intptr_t XMLCALL
        xmlTextWriterWriteVFormatDTDElement(xmlTextWriterPtr writer,
                                            const xmlChar * name,
                                            const char *format,
                                            va_list argptr);
    XMLPUBFUN intptr_t XMLCALL xmlTextWriterWriteDTDElement(xmlTextWriterPtr
                                                       writer,
                                                       const xmlChar *
                                                       name,
                                                       const xmlChar *
                                                       content);

/*
 * DTD attribute list definition
 */
    XMLPUBFUN intptr_t XMLCALL
        xmlTextWriterStartDTDAttlist(xmlTextWriterPtr writer,
                                     const xmlChar * name);
    XMLPUBFUN intptr_t XMLCALL xmlTextWriterEndDTDAttlist(xmlTextWriterPtr
                                                     writer);

/*
 * DTD attribute list definition conveniency functions
 */
    XMLPUBFUN intptr_t XMLCALL
        xmlTextWriterWriteFormatDTDAttlist(xmlTextWriterPtr writer,
                                           const xmlChar * name,
                                           const char *format, ...);
    XMLPUBFUN intptr_t XMLCALL
        xmlTextWriterWriteVFormatDTDAttlist(xmlTextWriterPtr writer,
                                            const xmlChar * name,
                                            const char *format,
                                            va_list argptr);
    XMLPUBFUN intptr_t XMLCALL xmlTextWriterWriteDTDAttlist(xmlTextWriterPtr
                                                       writer,
                                                       const xmlChar *
                                                       name,
                                                       const xmlChar *
                                                       content);

/*
 * DTD entity definition
 */
    XMLPUBFUN intptr_t XMLCALL
        xmlTextWriterStartDTDEntity(xmlTextWriterPtr writer,
                                    intptr_t pe, const xmlChar * name);
    XMLPUBFUN intptr_t XMLCALL xmlTextWriterEndDTDEntity(xmlTextWriterPtr
                                                    writer);

/*
 * DTD entity definition conveniency functions
 */
    XMLPUBFUN intptr_t XMLCALL
        xmlTextWriterWriteFormatDTDInternalEntity(xmlTextWriterPtr writer,
                                                  intptr_t pe,
                                                  const xmlChar * name,
                                                  const char *format, ...);
    XMLPUBFUN intptr_t XMLCALL
        xmlTextWriterWriteVFormatDTDInternalEntity(xmlTextWriterPtr writer,
                                                   intptr_t pe,
                                                   const xmlChar * name,
                                                   const char *format,
                                                   va_list argptr);
    XMLPUBFUN intptr_t XMLCALL
        xmlTextWriterWriteDTDInternalEntity(xmlTextWriterPtr writer,
                                            intptr_t pe,
                                            const xmlChar * name,
                                            const xmlChar * content);
    XMLPUBFUN intptr_t XMLCALL
        xmlTextWriterWriteDTDExternalEntity(xmlTextWriterPtr writer,
                                            intptr_t pe,
                                            const xmlChar * name,
                                            const xmlChar * pubid,
                                            const xmlChar * sysid,
                                            const xmlChar * ndataid);
    XMLPUBFUN intptr_t XMLCALL
        xmlTextWriterWriteDTDExternalEntityContents(xmlTextWriterPtr
                                                    writer,
                                                    const xmlChar * pubid,
                                                    const xmlChar * sysid,
                                                    const xmlChar *
                                                    ndataid);
    XMLPUBFUN intptr_t XMLCALL xmlTextWriterWriteDTDEntity(xmlTextWriterPtr
                                                      writer, intptr_t pe,
                                                      const xmlChar * name,
                                                      const xmlChar *
                                                      pubid,
                                                      const xmlChar *
                                                      sysid,
                                                      const xmlChar *
                                                      ndataid,
                                                      const xmlChar *
                                                      content);

/*
 * DTD notation definition
 */
    XMLPUBFUN intptr_t XMLCALL
        xmlTextWriterWriteDTDNotation(xmlTextWriterPtr writer,
                                      const xmlChar * name,
                                      const xmlChar * pubid,
                                      const xmlChar * sysid);

/*
 * Indentation
 */
    XMLPUBFUN intptr_t XMLCALL
        xmlTextWriterSetIndent(xmlTextWriterPtr writer, intptr_t indent);
    XMLPUBFUN intptr_t XMLCALL
        xmlTextWriterSetIndentString(xmlTextWriterPtr writer,
                                     const xmlChar * str);

/*
 * misc
 */
    XMLPUBFUN intptr_t XMLCALL xmlTextWriterFlush(xmlTextWriterPtr writer);

#ifdef __cplusplus
}
#endif

#endif /* LIBXML_WRITER_ENABLED */

#endif                          /* __XML_XMLWRITER_H__ */
