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
\r\n\
		<meta http-equiv="content-type" content="text/html; charset=UTF-8">\r\n\
		<meta charset="UTF-8">\r\n\
\r\n\
		<!-- Markdeep styles -->\r\n\
		<style>body{counter-reset: h1 h2 h3 h4 h5 h6}.md code,pre{font-family:Menlo,Consolas,monospace;font-size:15.018802542857143px;line-height:140%}.md div.title{font-size:26px;font-weight:800;line-height:120%;text-align:center}.md div.afterTitles{height:10px}.md div.subtitle{text-align:center}.md .image{display:inline-block}.md div.imagecaption,.md div.tablecaption,.md div.listingcaption{margin:0.2em 5px 10px 5px;text-align: justify;font-style:italic}.md div.imagecaption{margin-bottom:0}.md img{max-width:100%;page-break-inside:avoid}li{text-align:left};.md div.tilde{margin:20px 0 -10px;text-align:center}.md blockquote.fancyquote{margin:25px 0 25px;text-align:left;line-height:160%}.md blockquote.fancyquote::before{content:"“";color:#DDD;font-family:Times New Roman;font-size:45px;line-height:0;margin-right:6px;vertical-align:-0.3em}.md span.fancyquote{font-size:118%;color:#777;font-style:italic}.md span.fancyquote::after{content:"”";font-style:normal;color:#DDD;font-family:Times New Roman;font-size:45px;line-height:0;margin-left:6px;vertical-align:-0.3em}.md blockquote.fancyquote .author{width:100%;margin-top:10px;display:inline-block;text-align:right}.md small{font-size:60%}.md div.title,contents,.md .tocHeader,h1,h2,h3,h4,h5,h6,.md .shortTOC,.md .mediumTOC,.nonumberh1,.nonumberh2,.nonumberh3,.nonumberh4,.nonumberh5,.nonumberh6{font-family:Verdana,Helvetica,Arial,sans-serif;margin:13.4px 0 13.4px;padding:15px 0 3px;border-top:none;clear:both}.md svg.diagram{display:block;font-family:Menlo,Consolas,monospace;font-size:15.018802542857143px;text-align:center;stroke-linecap:round;stroke-width:2px;page-break-inside:avoid;stroke:#000;fill:#000}.md svg.diagram .opendot{fill:#FFF}.md svg.diagram text{stroke:none}.md h1,.tocHeader,.nonumberh1{border-bottom:3px solid;font-size:20px;font-weight:bold;}h1,.nonumberh1{counter-reset: h2 h3 h4 h5 h6}h2,.nonumberh2{counter-reset: h3 h4 h5 h6;border-bottom:2px solid #999;color:#555;font-size:18px;}h3,h4,h5,h6,.nonumberh3,.nonumberh4,.nonumberh5,.nonumberh6{font-family:Helvetica,Arial,sans-serif;color:#555;font-size:16px;}h3{counter-reset:h4 h5 h6}h4{counter-reset:h5 h6}h5{counter-reset:h6}.md table{border-collapse:collapse;line-height:140%;page-break-inside:avoid}.md table.table{margin:auto}.md table.calendar{width:100%;margin:auto;font-size:11px;font-family:Helvetica,Arial,sans-serif}.md table.calendar th{font-size:16px}.md .today{background:#ECF8FA}.md .calendar .parenthesized{color:#999;font-style:italic}.md div.tablecaption{text-align:center}.md table.table th{color:#FFF;background-color:#AAA;border:1px solid #888;padding:8px 15px 8px 15px}.md table.table td{padding:5px 15px 5px 15px;border:1px solid #888}.md table.table tr:nth-child(even){background:#EEE}.md pre.tilde{border-top: 1px solid #CCC;border-bottom: 1px solid #CCC;padding: 5px 0 5px 20px;margin:0 0 30px 0;background:#FCFCFC;page-break-inside:avoid}.md a:link, .md a:visited{color:#38A;text-decoration:none}.md a:link:hover{text-decoration:underline}.md dt{font-weight:700}dl>.md dd{padding:0 0 18px}.md dl>table{margin:35px 0 30px}.md code{white-space:pre;page-break-inside:avoid}.md .endnote{font-size:13px;line-height:15px;padding-left:10px;text-indent:-10px}.md .bib{padding-left:80px;text-indent:-80px;text-align:left}.markdeepFooter{font-size:9px;text-align:right;padding-top:80px;color:#999}.md .mediumTOC{float:right;font-size:12px;line-height:15px;border-left:1px solid #CCC;padding-left:15px;margin:15px 0px 15px 25px}.md .mediumTOC .level1{font-weight:600}.md .longTOC .level1{font-weight:600;display:block;padding-top:12px;margin:0 0 -20px}.md .shortTOC{text-align:center;font-weight:bold;margin-top:15px;font-size:14px}</style>\r\n\
\r\n\
		<!-- hljs styles -->\r\n\
		<style>.hljs{display:block;overflow-x:auto;padding:0.5em;background:#fff;color:#000;-webkit-text-size-adjust:none}.hljs-comment{color:#006a00}.hljs-keyword{color:#02E}.hljs-literal,.nginx .hljs-title{color:#aa0d91}.method,.hljs-list .hljs-title,.hljs-tag .hljs-title,.setting .hljs-value,.hljs-winutils,.tex .hljs-command,.http .hljs-title,.hljs-request,.hljs-status,.hljs-name{color:#008}.hljs-envvar,.tex .hljs-special{color:#660}.hljs-string{color:#c41a16}.hljs-tag .hljs-value,.hljs-cdata,.hljs-filter .hljs-argument,.hljs-attr_selector,.apache .hljs-cbracket,.hljs-date,.hljs-regexp{color:#080}.hljs-sub .hljs-identifier,.hljs-pi,.hljs-tag,.hljs-tag .hljs-keyword,.hljs-decorator,.ini .hljs-title,.hljs-shebang,.hljs-prompt,.hljs-hexcolor,.hljs-rule .hljs-value,.hljs-symbol,.hljs-symbol .hljs-string,.hljs-number,.css .hljs-function,.hljs-function .hljs-title,.coffeescript .hljs-attribute{color:#A0C}.hljs-function .hljs-title{font-weight:bold;color:#000}.hljs-class .hljs-title,.smalltalk .hljs-class,.hljs-type,.hljs-typename,.hljs-tag .hljs-attribute,.hljs-doctype,.hljs-class .hljs-id,.hljs-built_in,.setting,.hljs-params,.clojure .hljs-attribute{color:#5c2699}.hljs-variable{color:#3f6e74}.css .hljs-tag,.hljs-rule .hljs-property,.hljs-pseudo,.hljs-subst{color:#000}.css .hljs-class,.css .hljs-id{color:#9b703f}.hljs-value .hljs-important{color:#ff7700;font-weight:bold}.hljs-rule .hljs-keyword{color:#c5af75}.hljs-annotation,.apache .hljs-sqbracket,.nginx .hljs-built_in{color:#9b859d}.hljs-preprocessor,.hljs-preprocessor *,.hljs-pragma{color:#643820}.tex .hljs-formula{background-color:#eee;font-style:italic}.diff .hljs-header,.hljs-chunk{color:#808080;font-weight:bold}.diff .hljs-change{background-color:#bccff9}.hljs-addition{background-color:#baeeba}.hljs-deletion{background-color:#ffc8bd}.hljs-comment .hljs-doctag{font-weight:bold}.method .hljs-id{color:#000}</style>\r\n\
	\
';

// This code is placed at the beginning of the body before the Markdeep code. 
// $ (DOCUMENT_BODY_PREFIX) is everything in the body of PreviewBlogPage.htm up to 
// $ (ARTICLE_HTML_CODE).
DocumentBodyPrefix='\
\r\n\
\
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
\r\n\
	\
';

// Get the full Markdeep code from the .md.html file without the script invocation
MarkdeepCode=nodeToMarkdeepSource(document.body);
MarkdeepCode=MarkdeepCode.slice(0,MarkdeepCode.lastIndexOf("<script"));
// Bring it into a form where it can be pasted into an HTML document
SanitizedMarkdeepCode=escapeHTMLEntities(MarkdeepCode);
// Surround it by the prefix and suffix code and set that as body code
document.body.innerHTML=DocumentBodyPrefix+SanitizedMarkdeepCode+DocumentBodySuffix;

// Setting head attributes

// Setting body attributes
