DEVHELP_DIR=$(datadir)/gtk-doc/html/libxml2
HTML_FILES=index.html general.html $(HTML_MODULES)
HTML_MODULES=				\
	libxml2-c14n.html		\
	libxml2-catalog.html		\
	libxml2-chvalid.html		\
	libxml2-debugXML.html		\
	libxml2-dict.html		\
	libxml2-DOCBparser.html		\
	libxml2-encoding.html		\
	libxml2-entities.html		\
	libxml2-globals.html		\
	libxml2-hash.html		\
	libxml2-HTMLparser.html		\
	libxml2-HTMLtree.html		\
	libxml2-list.html		\
	libxml2-nanoftp.html		\
	libxml2-nanohttp.html		\
	libxml2-parser.html		\
	libxml2-parserInternals.html	\
	libxml2-pattern.html		\
	libxml2-relaxng.html		\
	libxml2-SAX2.html		\
	libxml2-SAX.html		\
	libxml2-schemasInternals.html	\
	libxml2-schematron.html		\
	libxml2-threads.html		\
	libxml2-tree.html		\
	libxml2-uri.html		\
	libxml2-valid.html		\
	libxml2-xinclude.html		\
	libxml2-xlink.html		\
	libxml2-xmlautomata.html	\
	libxml2-xmlerror.html		\
	libxml2-xmlexports.html		\
	libxml2-xmlIO.html		\
	libxml2-xmlmemory.html		\
	libxml2-xmlmodule.html		\
	libxml2-xmlreader.html		\
	libxml2-xmlregexp.html		\
	libxml2-xmlsave.html		\
	libxml2-xmlschemas.html		\
	libxml2-xmlschemastypes.html	\
	libxml2-xmlstring.html		\
	libxml2-xmlunicode.html		\
	libxml2-xmlversion.html		\
	libxml2-xmlwriter.html		\
	libxml2-xpath.html		\
	libxml2-xpathInternals.html	\
	libxml2-xpointer.html

EXTRA_FORMAT=				\
	home.png			\
	left.png			\
	right.png			\
	up.png				\
	style.css

EXTRA_DIST=devhelp.xsl html.xsl libxml2.devhelp $(HTML_FILES) $(EXTRA_FORMAT)

all: libxml2.devhelp $(HTML_FILES)

libxml2.devhelp $(HTML_FILES): devhelp.xsl html.xsl $(top_srcdir)/doc/libxml2-api.xml
	-@(echo Rebuilding devhelp files)
	-@(if [ -x $(XSLTPROC) ] ; then \
	  $(XSLTPROC) --nonet -o $(srcdir)/libxml2.devhelp devhelp.xsl $(top_srcdir)/doc/libxml2-api.xml ; fi );

install-data-local: 
	$(mkinstalldirs) $(DESTDIR)$(DEVHELP_DIR)
	-@INSTALL@ -m 0644 libxml2.devhelp $(DESTDIR)$(DEVHELP_DIR)
	-@INSTALL@ -m 0644 $(EXTRA_FORMAT) $(DESTDIR)$(DEVHELP_DIR)
	-@INSTALL@ -m 0644 $(HTML_FILES) $(DESTDIR)$(DEVHELP_DIR)

