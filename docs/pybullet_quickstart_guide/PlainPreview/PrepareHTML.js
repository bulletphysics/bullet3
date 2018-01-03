/** Converts <>&" to their HTML escape sequences */
function escapeHTMLEntities(str) {
    return String(str).replace(/&/g, '&amp;').replace(/</g, '&lt;').replace(/>/g, '&gt;').replace(/"/g, '&quot;');
}


/** Restores the original source string's '<' and '>' as entered in
    the document, before the browser processed it as HTML. There is no
    way in an HTML document to distinguish an entity that was entered
    as an entity.*/
function unescapeHTMLEntities(str) {
    // Process &amp; last so that we don't recursively unescape
    // escaped escape sequences.
    return str.
        replace(/&lt;/g, '<').
        replace(/&gt;/g, '>').
        replace(/&quot;/g, '"').
        replace(/&#39;/g, "'").
        replace(/&ndash;/g, '--').
        replace(/&mdash;/g, '---').
        replace(/&amp;/g, '&');
}


/**
   \param node  A node from an HTML DOM

   \return A String that is a very good reconstruction of what the
   original source looked like before the browser tried to correct
   it to legal HTML.
 */
function nodeToMarkdeepSource(node, leaveEscapes) {
    var source = node.innerHTML;

    // Markdown uses <john@bar.com> e-mail syntax, which HTML parsing
    // will try to close by inserting the matching close tags at the end of the
    // document. Remove anything that looks like that and comes *after*
    // the first fallback style.
    source = source.replace(/(?:<style class="fallback">[\s\S]*?<\/style>[\s\S]*)<\/\S+@\S+\.\S+?>/gim, '');
    
    // Remove artificially inserted close tags
    source = source.replace(/<\/h?ttps?:.*>/gi, '');
    
    // Now try to fix the URLs themselves, which will be 
    // transformed like this: <http: casual-effects.com="" markdeep="">
    source = source.replace(/<(https?): (.*?)>/gi, function (match, protocol, list) {

        // Remove any quotes--they wouldn't have been legal in the URL anyway
        var s = '<' + protocol + '://' + list.replace(/=""\s/g, '/');

        if (s.substring(s.length - 3) === '=""') {
            s = s.substring(0, s.length - 3);
        }

        // Remove any lingering quotes (since they
        // wouldn't have been legal in the URL)
        s = s.replace(/"/g, '');

        return s + '>';
    });

    // Remove the "fallback" style tags
    source = source.replace(/<style class=["']fallback["']>.*?<\/style>/gmi, '');

    source = unescapeHTMLEntities(source);

    return source;
}

// $ (FULL_DOCUMENT_HEAD) is replaced by the contents of the <head> found in 
// PreviewBlogPage.htm. This document head will overwrite whatever Markdeep does to 
// the head at the very end.
FullDocumentHead='\
$(FULL_DOCUMENT_HEAD)\
';

// This code is placed at the beginning of the body before the Markdeep code. 
// $ (DOCUMENT_BODY_PREFIX) is everything in the body of PreviewBlogPage.htm up to 
// $ (ARTICLE_HTML_CODE).
DocumentBodyPrefix='\
$(DOCUMENT_BODY_PREFIX)\
	<!-- MARKDEEP_BEGIN -->\
	<pre class="markdeep">\
';
// This code is placed at the end of the body after the Markdeep code. 
// $ (DOCUMENT_BODY_SUFFIX) is everything in the body of PreviewBlogPage.htm after 
// $ (ARTICLE_HTML_CODE).
DocumentBodySuffix='\
	</pre>\
	<!-- MARKDEEP_END -->\
	<div>Document &lt;body&gt; code:<br/>\
	<textarea cols="40" rows="10" id="BodyDisplayBox"></textarea></div>\
$(DOCUMENT_BODY_SUFFIX)\
';

// Get the full Markdeep code from the .md.html file without the script invocation
MarkdeepCode=nodeToMarkdeepSource(document.body);
MarkdeepCode=MarkdeepCode.slice(0,MarkdeepCode.lastIndexOf("<script"));
// Bring it into a form where it can be pasted into an HTML document
SanitizedMarkdeepCode=escapeHTMLEntities(MarkdeepCode);
// Surround it by the prefix and suffix code and set that as body code
document.body.innerHTML=DocumentBodyPrefix+SanitizedMarkdeepCode+DocumentBodySuffix;
