// PrepareHTML.full.js

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
	<meta name="viewport" content="width=device-width">\r\n\
\r\n\
	<!-- Markdeep styles -->\r\n\
	<style>body{counter-reset: h1 h2 h3 h4 h5 h6}.md code,pre{font-family:Menlo,Consolas,monospace;font-size:15.018802542857143px;line-height:140%}.md div.title{font-size:26px;font-weight:800;line-height:120%;text-align:center}.md div.afterTitles{height:10px}.md div.subtitle{text-align:center}.md .image{display:inline-block}.md div.imagecaption,.md div.tablecaption,.md div.listingcaption{margin:0.2em 5px 10px 5px;text-align: justify;font-style:italic}.md div.imagecaption{margin-bottom:0}.md img{max-width:100%;page-break-inside:avoid}li{text-align:left};.md div.tilde{margin:20px 0 -10px;text-align:center}.md blockquote.fancyquote{margin:25px 0 25px;text-align:left;line-height:160%}.md blockquote.fancyquote::before{content:"“";color:#DDD;font-family:Times New Roman;font-size:45px;line-height:0;margin-right:6px;vertical-align:-0.3em}.md span.fancyquote{font-size:118%;color:#777;font-style:italic}.md span.fancyquote::after{content:"”";font-style:normal;color:#DDD;font-family:Times New Roman;font-size:45px;line-height:0;margin-left:6px;vertical-align:-0.3em}.md blockquote.fancyquote .author{width:100%;margin-top:10px;display:inline-block;text-align:right}.md small{font-size:60%}.md div.title,contents,.md .tocHeader,h1,h2,h3,h4,h5,h6,.md .shortTOC,.md .mediumTOC,.nonumberh1,.nonumberh2,.nonumberh3,.nonumberh4,.nonumberh5,.nonumberh6{font-family:Verdana,Helvetica,Arial,sans-serif;margin:13.4px 0 13.4px;padding:15px 0 3px;border-top:none;clear:both}.md svg.diagram{display:block;font-family:Menlo,Consolas,monospace;font-size:15.018802542857143px;text-align:center;stroke-linecap:round;stroke-width:2px;page-break-inside:avoid;stroke:#000;fill:#000}.md svg.diagram .opendot{fill:#FFF}.md svg.diagram text{stroke:none}.md h1,.tocHeader,.nonumberh1{border-bottom:3px solid;font-size:20px;font-weight:bold;}h1,.nonumberh1{counter-reset: h2 h3 h4 h5 h6}h2,.nonumberh2{counter-reset: h3 h4 h5 h6;border-bottom:2px solid #999;color:#555;font-size:18px;}h3,h4,h5,h6,.nonumberh3,.nonumberh4,.nonumberh5,.nonumberh6{font-family:Helvetica,Arial,sans-serif;color:#555;font-size:16px;}h3{counter-reset:h4 h5 h6}h4{counter-reset:h5 h6}h5{counter-reset:h6}.md table{border-collapse:collapse;line-height:140%;page-break-inside:avoid}.md table.table{margin:auto}.md table.calendar{width:100%;margin:auto;font-size:11px;font-family:Helvetica,Arial,sans-serif}.md table.calendar th{font-size:16px}.md .today{background:#ECF8FA}.md .calendar .parenthesized{color:#999;font-style:italic}.md div.tablecaption{text-align:center}.md table.table th{color:#FFF;background-color:#AAA;border:1px solid #888;padding:8px 15px 8px 15px}.md table.table td{padding:5px 15px 5px 15px;border:1px solid #888}.md table.table tr:nth-child(even){background:#EEE}.md pre.tilde{border-top: 1px solid #CCC;border-bottom: 1px solid #CCC;padding: 5px 0 5px 20px;margin:0 0 30px 0;background:#FCFCFC;page-break-inside:avoid}.md a:link, .md a:visited{color:#38A;text-decoration:none}.md a:link:hover{text-decoration:underline}.md dt{font-weight:700}dl>.md dd{padding:0 0 18px}.md dl>table{margin:35px 0 30px}.md code{white-space:pre;page-break-inside:avoid}.md .endnote{font-size:13px;line-height:15px;padding-left:10px;text-indent:-10px}.md .bib{padding-left:80px;text-indent:-80px;text-align:left}.markdeepFooter{font-size:9px;text-align:right;padding-top:80px;color:#999}.md .mediumTOC{float:right;font-size:12px;line-height:15px;border-left:1px solid #CCC;padding-left:15px;margin:15px 0px 15px 25px}.md .mediumTOC .level1{font-weight:600}.md .longTOC .level1{font-weight:600;display:block;padding-top:12px;margin:0 0 -20px}.md .shortTOC{text-align:center;font-weight:bold;margin-top:15px;font-size:14px}</style>\r\n\
\r\n\
	<!-- hljs styles -->\r\n\
	<style>.hljs{display:block;overflow-x:auto;padding:0.5em;background:#fff;color:#000;-webkit-text-size-adjust:none}.hljs-comment{color:#006a00}.hljs-keyword{color:#02E}.hljs-literal,.nginx .hljs-title{color:#aa0d91}.method,.hljs-list .hljs-title,.hljs-tag .hljs-title,.setting .hljs-value,.hljs-winutils,.tex .hljs-command,.http .hljs-title,.hljs-request,.hljs-status,.hljs-name{color:#008}.hljs-envvar,.tex .hljs-special{color:#660}.hljs-string{color:#c41a16}.hljs-tag .hljs-value,.hljs-cdata,.hljs-filter .hljs-argument,.hljs-attr_selector,.apache .hljs-cbracket,.hljs-date,.hljs-regexp{color:#080}.hljs-sub .hljs-identifier,.hljs-pi,.hljs-tag,.hljs-tag .hljs-keyword,.hljs-decorator,.ini .hljs-title,.hljs-shebang,.hljs-prompt,.hljs-hexcolor,.hljs-rule .hljs-value,.hljs-symbol,.hljs-symbol .hljs-string,.hljs-number,.css .hljs-function,.hljs-function .hljs-title,.coffeescript .hljs-attribute{color:#A0C}.hljs-function .hljs-title{font-weight:bold;color:#000}.hljs-class .hljs-title,.smalltalk .hljs-class,.hljs-type,.hljs-typename,.hljs-tag .hljs-attribute,.hljs-doctype,.hljs-class .hljs-id,.hljs-built_in,.setting,.hljs-params,.clojure .hljs-attribute{color:#5c2699}.hljs-variable{color:#3f6e74}.css .hljs-tag,.hljs-rule .hljs-property,.hljs-pseudo,.hljs-subst{color:#000}.css .hljs-class,.css .hljs-id{color:#9b703f}.hljs-value .hljs-important{color:#ff7700;font-weight:bold}.hljs-rule .hljs-keyword{color:#c5af75}.hljs-annotation,.apache .hljs-sqbracket,.nginx .hljs-built_in{color:#9b859d}.hljs-preprocessor,.hljs-preprocessor *,.hljs-pragma{color:#643820}.tex .hljs-formula{background-color:#eee;font-style:italic}.diff .hljs-header,.hljs-chunk{color:#808080;font-weight:bold}.diff .hljs-change{background-color:#bccff9}.hljs-addition{background-color:#baeeba}.hljs-deletion{background-color:#ffc8bd}.hljs-comment .hljs-doctag{font-weight:bold}.method .hljs-id{color:#000}</style>\r\n\
\r\n\
	<style type="text/css">\r\n\
img.wp-smiley,\r\n\
img.emoji {\r\n\
	display: inline !important;\r\n\
	border: none !important;\r\n\
	box-shadow: none !important;\r\n\
	height: 1em !important;\r\n\
	width: 1em !important;\r\n\
	margin: 0 .07em !important;\r\n\
	vertical-align: -0.1em !important;\r\n\
	background: none !important;\r\n\
	padding: 0 !important;\r\n\
}\r\n\
</style>\r\n\
\r\n\
<style type="text/css">\r\n\
/*\r\n\
Theme Name: Twenty Fourteen\r\n\
Theme URI: https://wordpress.org/themes/twentyfourteen/\r\n\
Author: the WordPress team\r\n\
Author URI: https://wordpress.org/\r\n\
Description: In 2014, our default theme lets you create a responsive magazine website with a sleek, modern design. Feature your favorite homepage content in either a grid or a slider. Use the three widget areas to customize your website, and change your content\'s layout with a full-width page template and a contributor page to show off your authors. Creating a magazine website with WordPress has never been easier.\r\n\
Version: 1.7\r\n\
License: GNU General Public License v2 or later\r\n\
License URI: http://www.gnu.org/licenses/gpl-2.0.html\r\n\
Tags: black, green, white, light, dark, two-columns, three-columns, left-sidebar, right-sidebar, fixed-layout, responsive-layout, custom-background, custom-header, custom-menu, editor-style, featured-images, flexible-header, full-width-template, microformats, post-formats, rtl-language-support, sticky-post, theme-options, translation-ready, accessibility-ready\r\n\
Text Domain: twentyfourteen\r\n\
\r\n\
This theme, like WordPress, is licensed under the GPL.\r\n\
Use it to make something cool, have fun, and share what you\'ve learned with others.\r\n\
*/\r\n\
\r\n\
/**\r\n\
 * Table of Contents:\r\n\
 *\r\n\
 * 1.0 - Reset\r\n\
 * 2.0 - Repeatable Patterns\r\n\
 * 3.0 - Basic Structure\r\n\
 * 4.0 - Header\r\n\
 * 5.0 - Navigation\r\n\
 * 6.0 - Content\r\n\
 *   6.1 - Post Thumbnail\r\n\
 *   6.2 - Entry Header\r\n\
 *   6.3 - Entry Meta\r\n\
 *   6.4 - Entry Content\r\n\
 *   6.5 - Galleries\r\n\
 *   6.6 - Post Formats\r\n\
 *   6.7 - Post/Image/Paging Navigation\r\n\
 *   6.8 - Attachments\r\n\
 *   6.9 - Archives\r\n\
 *   6.10 - Contributor Page\r\n\
 *   6.11 - 404 Page\r\n\
 *   6.12 - Full-width\r\n\
 *   6.13 - Singular\r\n\
 *   6.14 - Comments\r\n\
 * 7.0 - Sidebar\r\n\
 *   7.1 - Widgets\r\n\
 *   7.2 - Content Sidebar Widgets\r\n\
 * 8.0 - Footer\r\n\
 * 9.0 - Featured Content\r\n\
 * 10.0 - Multisite\r\n\
 * 11.0 - Media Queries\r\n\
 * 12.0 - Print\r\n\
 * -----------------------------------------------------------------------------\r\n\
 */\r\n\
\r\n\
\r\n\
/**\r\n\
 * 1.0 Reset\r\n\
 *\r\n\
 * Resetting and rebuilding styles have been helped along thanks to the fine\r\n\
 * work of Eric Meyer, Nicolas Gallagher, Jonathan Neal, and Blueprint.\r\n\
 *\r\n\
 * -----------------------------------------------------------------------------\r\n\
 */\r\n\
\r\n\
html, body, div, span, applet, object, iframe, h1, h2, h3, h4, h5, h6, p, blockquote, pre, a, abbr, acronym, address, big, cite, code, del, dfn, em, font, ins, kbd, q, s, samp, small, strike, strong, sub, sup, tt, var, dl, dt, dd, ol, ul, li, fieldset, form, label, legend, table, caption, tbody, tfoot, thead, tr, th, td {\r\n\
	border: 0;\r\n\
	font-family: inherit;\r\n\
	font-size: 100%;\r\n\
	font-style: inherit;\r\n\
	font-weight: inherit;\r\n\
	margin: 0;\r\n\
	outline: 0;\r\n\
	padding: 0;\r\n\
	vertical-align: baseline;\r\n\
}\r\n\
\r\n\
article,\r\n\
aside,\r\n\
details,\r\n\
figcaption,\r\n\
figure,\r\n\
footer,\r\n\
header,\r\n\
hgroup,\r\n\
nav,\r\n\
section {\r\n\
	display: block;\r\n\
}\r\n\
\r\n\
audio,\r\n\
canvas,\r\n\
video {\r\n\
	display: inline-block;\r\n\
	max-width: 100%;\r\n\
}\r\n\
\r\n\
html {\r\n\
	overflow-y: scroll;\r\n\
	-webkit-text-size-adjust: 100%;\r\n\
	-ms-text-size-adjust:     100%;\r\n\
}\r\n\
\r\n\
body,\r\n\
button,\r\n\
input,\r\n\
select,\r\n\
textarea {\r\n\
	color: #2b2b2b;\r\n\
	font-family: Lato, sans-serif;\r\n\
	font-size: 16px;\r\n\
	font-weight: 400;\r\n\
	line-height: 1.5;\r\n\
}\r\n\
\r\n\
body {\r\n\
	background: #f5f5f5;\r\n\
}\r\n\
\r\n\
a {\r\n\
	color: #24890d;\r\n\
	text-decoration: none;\r\n\
}\r\n\
\r\n\
a:focus {\r\n\
	outline: thin dotted;\r\n\
}\r\n\
\r\n\
a:hover,\r\n\
a:active {\r\n\
	outline: 0;\r\n\
}\r\n\
\r\n\
a:active,\r\n\
a:hover {\r\n\
	color: #41a62a;\r\n\
}\r\n\
\r\n\
h1,\r\n\
h2,\r\n\
h3,\r\n\
h4,\r\n\
h5,\r\n\
h6 {\r\n\
	clear: both;\r\n\
	font-weight: 700;\r\n\
	margin: 36px 0 12px;\r\n\
}\r\n\
\r\n\
h1 {\r\n\
	font-size: 26px;\r\n\
	line-height: 1.3846153846;\r\n\
}\r\n\
\r\n\
h2 {\r\n\
	font-size: 24px;\r\n\
	line-height: 1;\r\n\
}\r\n\
\r\n\
h3 {\r\n\
	font-size: 22px;\r\n\
	line-height: 1.0909090909;\r\n\
}\r\n\
\r\n\
h4 {\r\n\
	font-size: 20px;\r\n\
	line-height: 1.2;\r\n\
}\r\n\
\r\n\
h5 {\r\n\
	font-size: 18px;\r\n\
	line-height: 1.3333333333;\r\n\
}\r\n\
\r\n\
h6 {\r\n\
	font-size: 16px;\r\n\
	line-height: 1.5;\r\n\
}\r\n\
\r\n\
address {\r\n\
	font-style: italic;\r\n\
	margin-bottom: 24px;\r\n\
}\r\n\
\r\n\
abbr[title] {\r\n\
	border-bottom: 1px dotted #2b2b2b;\r\n\
	cursor: help;\r\n\
}\r\n\
\r\n\
b,\r\n\
strong {\r\n\
	font-weight: 700;\r\n\
}\r\n\
\r\n\
cite,\r\n\
dfn,\r\n\
em,\r\n\
i {\r\n\
	font-style: italic;\r\n\
}\r\n\
\r\n\
mark,\r\n\
ins {\r\n\
	background: #fff9c0;\r\n\
	text-decoration: none;\r\n\
}\r\n\
\r\n\
p {\r\n\
	margin-bottom: 24px;\r\n\
}\r\n\
\r\n\
code,\r\n\
kbd,\r\n\
tt,\r\n\
var,\r\n\
samp,\r\n\
pre {\r\n\
	font-family: monospace, serif;\r\n\
	font-size: 15px;\r\n\
	-webkit-hyphens: none;\r\n\
	-moz-hyphens:    none;\r\n\
	-ms-hyphens:     none;\r\n\
	hyphens:         none;\r\n\
	line-height: 1.6;\r\n\
}\r\n\
\r\n\
pre {\r\n\
	border: 1px solid rgba(0, 0, 0, 0.1);\r\n\
	-webkit-box-sizing: border-box;\r\n\
	-moz-box-sizing:    border-box;\r\n\
	box-sizing:         border-box;\r\n\
	margin-bottom: 24px;\r\n\
	max-width: 100%;\r\n\
	overflow: auto;\r\n\
	padding: 12px;\r\n\
	white-space: pre;\r\n\
	white-space: pre-wrap;\r\n\
	word-wrap: break-word;\r\n\
}\r\n\
\r\n\
blockquote,\r\n\
q {\r\n\
	-webkit-hyphens: none;\r\n\
	-moz-hyphens:    none;\r\n\
	-ms-hyphens:     none;\r\n\
	hyphens:         none;\r\n\
	quotes: none;\r\n\
}\r\n\
\r\n\
blockquote:before,\r\n\
blockquote:after,\r\n\
q:before,\r\n\
q:after {\r\n\
	content: "";\r\n\
	content: none;\r\n\
}\r\n\
\r\n\
blockquote {\r\n\
	color: #767676;\r\n\
	font-size: 19px;\r\n\
	font-style: italic;\r\n\
	font-weight: 300;\r\n\
	line-height: 1.2631578947;\r\n\
	margin-bottom: 24px;\r\n\
}\r\n\
\r\n\
blockquote cite,\r\n\
blockquote small {\r\n\
	color: #2b2b2b;\r\n\
	font-size: 16px;\r\n\
	font-weight: 400;\r\n\
	line-height: 1.5;\r\n\
}\r\n\
\r\n\
blockquote em,\r\n\
blockquote i,\r\n\
blockquote cite {\r\n\
	font-style: normal;\r\n\
}\r\n\
\r\n\
blockquote strong,\r\n\
blockquote b {\r\n\
	font-weight: 400;\r\n\
}\r\n\
\r\n\
small {\r\n\
	font-size: smaller;\r\n\
}\r\n\
\r\n\
big {\r\n\
	font-size: 125%;\r\n\
}\r\n\
\r\n\
sup,\r\n\
sub {\r\n\
	font-size: 75%;\r\n\
	height: 0;\r\n\
	line-height: 0;\r\n\
	position: relative;\r\n\
	vertical-align: baseline;\r\n\
}\r\n\
\r\n\
sup {\r\n\
	bottom: 1ex;\r\n\
}\r\n\
\r\n\
sub {\r\n\
	top: .5ex;\r\n\
}\r\n\
\r\n\
dl {\r\n\
	margin-bottom: 24px;\r\n\
}\r\n\
\r\n\
dt {\r\n\
	font-weight: bold;\r\n\
}\r\n\
\r\n\
dd {\r\n\
	margin-bottom: 24px;\r\n\
}\r\n\
\r\n\
ul,\r\n\
ol {\r\n\
	list-style: none;\r\n\
	margin: 0 0 24px 20px;\r\n\
}\r\n\
\r\n\
ul {\r\n\
	list-style: disc;\r\n\
}\r\n\
\r\n\
ol {\r\n\
	list-style: decimal;\r\n\
}\r\n\
\r\n\
li > ul,\r\n\
li > ol {\r\n\
	margin: 0 0 0 20px;\r\n\
}\r\n\
\r\n\
img {\r\n\
	-ms-interpolation-mode: bicubic;\r\n\
	border: 0;\r\n\
	vertical-align: middle;\r\n\
}\r\n\
\r\n\
figure {\r\n\
	margin: 0;\r\n\
}\r\n\
\r\n\
fieldset {\r\n\
	border: 1px solid rgba(0, 0, 0, 0.1);\r\n\
	margin: 0 0 24px;\r\n\
	min-width: inherit;\r\n\
	padding: 11px 12px 0;\r\n\
}\r\n\
\r\n\
legend {\r\n\
	white-space: normal;\r\n\
}\r\n\
\r\n\
button,\r\n\
input,\r\n\
select,\r\n\
textarea {\r\n\
	-webkit-box-sizing: border-box;\r\n\
	-moz-box-sizing:    border-box;\r\n\
	box-sizing:         border-box;\r\n\
	font-size: 100%;\r\n\
	margin: 0;\r\n\
	max-width: 100%;\r\n\
	vertical-align: baseline;\r\n\
}\r\n\
\r\n\
button,\r\n\
input {\r\n\
	line-height: normal;\r\n\
}\r\n\
\r\n\
input,\r\n\
textarea {\r\n\
	background-image: -webkit-linear-gradient(hsla(0,0%,100%,0), hsla(0,0%,100%,0)); /* Removing the inner shadow, rounded corners on iOS inputs */\r\n\
}\r\n\
\r\n\
button,\r\n\
html input[type="button"],\r\n\
input[type="reset"],\r\n\
input[type="submit"] {\r\n\
	-webkit-appearance: button;\r\n\
	cursor: pointer;\r\n\
}\r\n\
\r\n\
button[disabled],\r\n\
input[disabled] {\r\n\
	cursor: default;\r\n\
}\r\n\
\r\n\
input[type="checkbox"],\r\n\
input[type="radio"] {\r\n\
	padding: 0;\r\n\
}\r\n\
\r\n\
input[type="search"] {\r\n\
	-webkit-appearance: textfield;\r\n\
}\r\n\
\r\n\
input[type="search"]::-webkit-search-decoration {\r\n\
	-webkit-appearance: none;\r\n\
}\r\n\
\r\n\
button::-moz-focus-inner,\r\n\
input::-moz-focus-inner {\r\n\
	border: 0;\r\n\
	padding: 0;\r\n\
}\r\n\
\r\n\
textarea {\r\n\
	overflow: auto;\r\n\
	vertical-align: top;\r\n\
}\r\n\
\r\n\
table,\r\n\
th,\r\n\
td {\r\n\
	border: 1px solid rgba(0, 0, 0, 0.1);\r\n\
}\r\n\
\r\n\
table {\r\n\
	border-collapse: separate;\r\n\
	border-spacing: 0;\r\n\
	border-width: 1px 0 0 1px;\r\n\
	margin-bottom: 24px;\r\n\
	width: 100%;\r\n\
}\r\n\
\r\n\
caption,\r\n\
th,\r\n\
td {\r\n\
	font-weight: normal;\r\n\
	text-align: left;\r\n\
}\r\n\
\r\n\
th {\r\n\
	border-width: 0 1px 1px 0;\r\n\
	font-weight: bold;\r\n\
}\r\n\
\r\n\
td {\r\n\
	border-width: 0 1px 1px 0;\r\n\
}\r\n\
\r\n\
del {\r\n\
	color: #767676;\r\n\
}\r\n\
\r\n\
hr {\r\n\
	background-color: rgba(0, 0, 0, 0.1);\r\n\
	border: 0;\r\n\
	height: 1px;\r\n\
	margin-bottom: 23px;\r\n\
}\r\n\
\r\n\
/* Support a widely-adopted but non-standard selector for text selection styles\r\n\
 * to achieve a better experience. See https://core.trac.wordpress.org/ticket/25898.\r\n\
 */\r\n\
::selection {\r\n\
	background: #24890d;\r\n\
	color: #fff;\r\n\
	text-shadow: none;\r\n\
}\r\n\
\r\n\
::-moz-selection {\r\n\
	background: #24890d;\r\n\
	color: #fff;\r\n\
	text-shadow: none;\r\n\
}\r\n\
\r\n\
\r\n\
/**\r\n\
 * 2.0 Repeatable Patterns\r\n\
 * -----------------------------------------------------------------------------\r\n\
 */\r\n\
\r\n\
/* Input fields */\r\n\
\r\n\
input,\r\n\
textarea {\r\n\
	border: 1px solid rgba(0, 0, 0, 0.1);\r\n\
	border-radius: 2px;\r\n\
	color: #2b2b2b;\r\n\
	padding: 8px 10px 7px;\r\n\
}\r\n\
\r\n\
textarea {\r\n\
	width: 100%;\r\n\
}\r\n\
\r\n\
input:focus,\r\n\
textarea:focus {\r\n\
	border: 1px solid rgba(0, 0, 0, 0.3);\r\n\
	outline: 0;\r\n\
}\r\n\
\r\n\
/* Buttons */\r\n\
\r\n\
button,\r\n\
.button,\r\n\
input[type="button"],\r\n\
input[type="reset"],\r\n\
input[type="submit"] {\r\n\
	background-color: #24890d;\r\n\
	border: 0;\r\n\
	border-radius: 2px;\r\n\
	color: #fff;\r\n\
	font-size: 12px;\r\n\
	font-weight: 700;\r\n\
	padding: 10px 30px 11px;\r\n\
	text-transform: uppercase;\r\n\
	vertical-align: bottom;\r\n\
}\r\n\
\r\n\
button:hover,\r\n\
button:focus,\r\n\
.button:hover,\r\n\
.button:focus,\r\n\
input[type="button"]:hover,\r\n\
input[type="button"]:focus,\r\n\
input[type="reset"]:hover,\r\n\
input[type="reset"]:focus,\r\n\
input[type="submit"]:hover,\r\n\
input[type="submit"]:focus {\r\n\
	background-color: #41a62a;\r\n\
	color: #fff;\r\n\
}\r\n\
\r\n\
button:active,\r\n\
.button:active,\r\n\
input[type="button"]:active,\r\n\
input[type="reset"]:active,\r\n\
input[type="submit"]:active {\r\n\
	background-color: #55d737;\r\n\
}\r\n\
\r\n\
.search-field {\r\n\
	width: 100%;\r\n\
}\r\n\
\r\n\
.search-submit {\r\n\
	display: none;\r\n\
}\r\n\
\r\n\
/* Placeholder text color -- selectors need to be separate to work. */\r\n\
\r\n\
::-webkit-input-placeholder {\r\n\
	color: #939393;\r\n\
}\r\n\
\r\n\
:-moz-placeholder {\r\n\
	color: #939393;\r\n\
}\r\n\
\r\n\
::-moz-placeholder {\r\n\
	color: #939393;\r\n\
	opacity: 1; /* Since FF19 lowers the opacity of the placeholder by default */\r\n\
}\r\n\
\r\n\
:-ms-input-placeholder {\r\n\
	color: #939393;\r\n\
}\r\n\
\r\n\
/* Responsive images. Fluid images for posts, comments, and widgets */\r\n\
\r\n\
.comment-content img,\r\n\
.entry-content img,\r\n\
.entry-summary img,\r\n\
#site-header img,\r\n\
.widget img,\r\n\
.wp-caption {\r\n\
	max-width: 100%;\r\n\
}\r\n\
\r\n\
/**\r\n\
 * Make sure images with WordPress-added height and width attributes are\r\n\
 * scaled correctly.\r\n\
 */\r\n\
\r\n\
.comment-content img[height],\r\n\
.entry-content img,\r\n\
.entry-summary img,\r\n\
img[class*="align"],\r\n\
img[class*="wp-image-"],\r\n\
img[class*="attachment-"],\r\n\
#site-header img {\r\n\
	height: auto;\r\n\
}\r\n\
\r\n\
img.size-full,\r\n\
img.size-large,\r\n\
.wp-post-image,\r\n\
.post-thumbnail img {\r\n\
	height: auto;\r\n\
	max-width: 100%;\r\n\
}\r\n\
\r\n\
/* Make sure embeds and iframes fit their containers */\r\n\
\r\n\
embed,\r\n\
iframe,\r\n\
object,\r\n\
video {\r\n\
	margin-bottom: 24px;\r\n\
	max-width: 100%;\r\n\
}\r\n\
\r\n\
p > embed,\r\n\
p > iframe,\r\n\
p > object,\r\n\
span > embed,\r\n\
span > iframe,\r\n\
span > object {\r\n\
	margin-bottom: 0;\r\n\
}\r\n\
\r\n\
/* Alignment */\r\n\
\r\n\
.alignleft {\r\n\
	float: left;\r\n\
}\r\n\
\r\n\
.alignright {\r\n\
	float: right;\r\n\
}\r\n\
\r\n\
.aligncenter {\r\n\
	display: block;\r\n\
	margin-left: auto;\r\n\
	margin-right: auto;\r\n\
}\r\n\
\r\n\
blockquote.alignleft,\r\n\
figure.wp-caption.alignleft,\r\n\
img.alignleft {\r\n\
	margin: 7px 24px 7px 0;\r\n\
}\r\n\
\r\n\
.wp-caption.alignleft {\r\n\
	margin: 7px 14px 7px 0;\r\n\
}\r\n\
\r\n\
blockquote.alignright,\r\n\
figure.wp-caption.alignright,\r\n\
img.alignright {\r\n\
	margin: 7px 0 7px 24px;\r\n\
}\r\n\
\r\n\
.wp-caption.alignright {\r\n\
	margin: 7px 0 7px 14px;\r\n\
}\r\n\
\r\n\
blockquote.aligncenter,\r\n\
img.aligncenter,\r\n\
.wp-caption.aligncenter {\r\n\
	margin-top: 7px;\r\n\
	margin-bottom: 7px;\r\n\
}\r\n\
\r\n\
.site-content blockquote.alignleft,\r\n\
.site-content blockquote.alignright {\r\n\
	border-top: 1px solid rgba(0, 0, 0, 0.1);\r\n\
	border-bottom: 1px solid rgba(0, 0, 0, 0.1);\r\n\
	padding-top: 17px;\r\n\
	width: 50%;\r\n\
}\r\n\
\r\n\
.site-content blockquote.alignleft p,\r\n\
.site-content blockquote.alignright p {\r\n\
	margin-bottom: 17px;\r\n\
}\r\n\
\r\n\
.wp-caption {\r\n\
	margin-bottom: 24px;\r\n\
}\r\n\
\r\n\
.wp-caption img[class*="wp-image-"] {\r\n\
	display: block;\r\n\
	margin: 0;\r\n\
}\r\n\
\r\n\
.wp-caption {\r\n\
	color: #767676;\r\n\
}\r\n\
\r\n\
.wp-caption-text {\r\n\
	-webkit-box-sizing: border-box;\r\n\
	-moz-box-sizing:    border-box;\r\n\
	box-sizing:         border-box;\r\n\
	font-size: 12px;\r\n\
	font-style: italic;\r\n\
	line-height: 1.5;\r\n\
	margin: 9px 0;\r\n\
}\r\n\
\r\n\
div.wp-caption .wp-caption-text {\r\n\
	padding-right: 10px;\r\n\
}\r\n\
\r\n\
div.wp-caption.alignright img[class*="wp-image-"],\r\n\
div.wp-caption.alignright .wp-caption-text {\r\n\
	padding-left: 10px;\r\n\
	padding-right: 0;\r\n\
}\r\n\
\r\n\
.wp-smiley {\r\n\
	border: 0;\r\n\
	margin-bottom: 0;\r\n\
	margin-top: 0;\r\n\
	padding: 0;\r\n\
}\r\n\
\r\n\
/* Assistive text */\r\n\
\r\n\
.screen-reader-text {\r\n\
	clip: rect(1px, 1px, 1px, 1px);\r\n\
	overflow: hidden;\r\n\
	position: absolute !important;\r\n\
	height: 1px;\r\n\
	width: 1px;\r\n\
}\r\n\
\r\n\
.screen-reader-text:focus {\r\n\
	background-color: #f1f1f1;\r\n\
	border-radius: 3px;\r\n\
	box-shadow: 0 0 2px 2px rgba(0, 0, 0, 0.6);\r\n\
	clip: auto;\r\n\
	color: #21759b;\r\n\
	display: block;\r\n\
	font-size: 14px;\r\n\
	font-weight: bold;\r\n\
	height: auto;\r\n\
	line-height: normal;\r\n\
	padding: 15px 23px 14px;\r\n\
	position: absolute;\r\n\
	left: 5px;\r\n\
	top: 5px;\r\n\
	text-decoration: none;\r\n\
	text-transform: none;\r\n\
	width: auto;\r\n\
	z-index: 100000; /* Above WP toolbar */\r\n\
}\r\n\
\r\n\
.hide {\r\n\
	display: none;\r\n\
}\r\n\
\r\n\
/* Clearing floats */\r\n\
\r\n\
.footer-sidebar:before,\r\n\
.footer-sidebar:after,\r\n\
.hentry:before,\r\n\
.hentry:after,\r\n\
.gallery:before,\r\n\
.gallery:after,\r\n\
.slider-direction-nav:before,\r\n\
.slider-direction-nav:after,\r\n\
.contributor-info:before,\r\n\
.contributor-info:after,\r\n\
.search-box:before,\r\n\
.search-box:after,\r\n\
[class*="content"]:before,\r\n\
[class*="content"]:after,\r\n\
[class*="site"]:before,\r\n\
[class*="site"]:after {\r\n\
	content: "";\r\n\
	display: table;\r\n\
}\r\n\
\r\n\
.footer-sidebar:after,\r\n\
.hentry:after,\r\n\
.gallery:after,\r\n\
.slider-direction-nav:after,\r\n\
.contributor-info:after,\r\n\
.search-box:after,\r\n\
[class*="content"]:after,\r\n\
[class*="site"]:after {\r\n\
	clear: both;\r\n\
}\r\n\
\r\n\
/* Genericons */\r\n\
\r\n\
.bypostauthor > article .fn:before,\r\n\
.comment-edit-link:before,\r\n\
.comment-reply-link:before,\r\n\
.comment-reply-login:before,\r\n\
.comment-reply-title small a:before,\r\n\
.contributor-posts-link:before,\r\n\
.menu-toggle:before,\r\n\
.search-toggle:before,\r\n\
.slider-direction-nav a:before,\r\n\
.widget_twentyfourteen_ephemera .widget-title:before {\r\n\
	-webkit-font-smoothing: antialiased;\r\n\
	display: inline-block;\r\n\
	font: normal 16px/1 Genericons;\r\n\
	text-decoration: inherit;\r\n\
	vertical-align: text-bottom;\r\n\
}\r\n\
\r\n\
/* Separators */\r\n\
\r\n\
.site-content span + .entry-date:before,\r\n\
.full-size-link:before,\r\n\
.parent-post-link:before,\r\n\
span + .byline:before,\r\n\
span + .comments-link:before,\r\n\
span + .edit-link:before,\r\n\
.widget_twentyfourteen_ephemera .entry-title:after {\r\n\
	content: "\0020\007c\0020";\r\n\
}\r\n\
\r\n\
\r\n\
/**\r\n\
 * 3.0 Basic Structure\r\n\
 * -----------------------------------------------------------------------------\r\n\
 */\r\n\
\r\n\
.site {\r\n\
	background-color: #fff;\r\n\
	max-width: 1260px;\r\n\
	position: relative;\r\n\
}\r\n\
\r\n\
.main-content {\r\n\
	width: 100%;\r\n\
}\r\n\
\r\n\
\r\n\
/**\r\n\
 * 4.0 Header\r\n\
 * -----------------------------------------------------------------------------\r\n\
 */\r\n\
\r\n\
/* Ensure that there is no gap between the header and\r\n\
	 the admin bar for WordPress versions before 3.8. */\r\n\
#wpadminbar {\r\n\
	min-height: 32px;\r\n\
}\r\n\
\r\n\
#site-header {\r\n\
	position: relative;\r\n\
	z-index: 3;\r\n\
}\r\n\
\r\n\
.site-header {\r\n\
	background-color: #000;\r\n\
	max-width: 1260px;\r\n\
	position: relative;\r\n\
	width: 100%;\r\n\
	z-index: 4;\r\n\
}\r\n\
\r\n\
.header-main {\r\n\
	min-height: 48px;\r\n\
	padding: 0 10px;\r\n\
}\r\n\
\r\n\
.site-title {\r\n\
	float: left;\r\n\
	font-size: 18px;\r\n\
	font-weight: 700;\r\n\
	line-height: 48px;\r\n\
	margin: 0;\r\n\
\r\n\
	/* Nav-toggle width + search-toggle width - gutter = 86px */\r\n\
	max-width: -webkit-calc(100% - 86px);\r\n\
	max-width:         calc(100% - 86px);\r\n\
}\r\n\
\r\n\
.site-title a,\r\n\
.site-title a:hover {\r\n\
	color: #fff;\r\n\
	display: block;\r\n\
	overflow: hidden;\r\n\
	text-overflow: ellipsis;\r\n\
	white-space: nowrap;\r\n\
}\r\n\
\r\n\
/* Search in the header */\r\n\
\r\n\
.search-toggle {\r\n\
	background-color: #24890d;\r\n\
	cursor: pointer;\r\n\
	float: right;\r\n\
	height: 48px;\r\n\
	margin-right: 38px;\r\n\
	text-align: center;\r\n\
	width: 48px;\r\n\
}\r\n\
\r\n\
.search-toggle:hover,\r\n\
.search-toggle.active {\r\n\
	background-color: #41a62a;\r\n\
}\r\n\
\r\n\
.search-toggle:before {\r\n\
	color: #fff;\r\n\
	content: "\f400";\r\n\
	font-size: 20px;\r\n\
	margin-top: 14px;\r\n\
}\r\n\
\r\n\
.search-toggle .screen-reader-text {\r\n\
	left: 5px; /* Avoid a horizontal scrollbar when the site has a long menu */\r\n\
}\r\n\
\r\n\
.search-box-wrapper {\r\n\
	-webkit-box-sizing: border-box;\r\n\
	-moz-box-sizing:    border-box;\r\n\
	box-sizing:         border-box;\r\n\
	position: absolute;\r\n\
	top: 48px;\r\n\
	right: 0;\r\n\
	width: 100%;\r\n\
	z-index: 2;\r\n\
}\r\n\
\r\n\
.search-box {\r\n\
	background-color: #41a62a;\r\n\
	padding: 12px;\r\n\
}\r\n\
\r\n\
.search-box .search-field {\r\n\
	background-color: #fff;\r\n\
	border: 0;\r\n\
	float: right;\r\n\
	font-size: 16px;\r\n\
	padding: 2px 2px 3px 6px;\r\n\
	width: 100%;\r\n\
}\r\n\
\r\n\
\r\n\
/**\r\n\
 * 5.0 Navigation\r\n\
 * -----------------------------------------------------------------------------\r\n\
 */\r\n\
\r\n\
.site-navigation ul {\r\n\
	list-style: none;\r\n\
	margin: 0;\r\n\
}\r\n\
\r\n\
.site-navigation li {\r\n\
	border-top: 1px solid rgba(255, 255, 255, 0.2);\r\n\
}\r\n\
\r\n\
.site-navigation ul ul {\r\n\
	margin-left: 20px;\r\n\
}\r\n\
\r\n\
.site-navigation a {\r\n\
	color: #fff;\r\n\
	display: block;\r\n\
	text-transform: uppercase;\r\n\
}\r\n\
\r\n\
.site-navigation a:hover {\r\n\
	color: #41a62a;\r\n\
}\r\n\
\r\n\
.site-navigation .current_page_item > a,\r\n\
.site-navigation .current_page_ancestor > a,\r\n\
.site-navigation .current-menu-item > a,\r\n\
.site-navigation .current-menu-ancestor > a {\r\n\
	color: #55d737;\r\n\
	font-weight: 900;\r\n\
}\r\n\
\r\n\
/* Primary Navigation */\r\n\
\r\n\
.primary-navigation {\r\n\
	-webkit-box-sizing: border-box;\r\n\
	-moz-box-sizing:    border-box;\r\n\
	box-sizing:         border-box;\r\n\
	font-size: 14px;\r\n\
	padding-top: 24px;\r\n\
}\r\n\
\r\n\
.primary-navigation.toggled-on {\r\n\
	padding: 72px 0 36px;\r\n\
}\r\n\
\r\n\
.primary-navigation .nav-menu {\r\n\
	border-bottom: 1px solid rgba(255, 255, 255, 0.2);\r\n\
	display: none;\r\n\
}\r\n\
\r\n\
.primary-navigation.toggled-on .nav-menu {\r\n\
	display: block;\r\n\
}\r\n\
\r\n\
.primary-navigation a {\r\n\
	padding: 7px 0;\r\n\
}\r\n\
\r\n\
/* Secondary Navigation */\r\n\
\r\n\
.secondary-navigation {\r\n\
	border-bottom: 1px solid rgba(255, 255, 255, 0.2);\r\n\
	font-size: 12px;\r\n\
	margin: 48px 0;\r\n\
}\r\n\
\r\n\
.secondary-navigation a {\r\n\
	padding: 9px 0;\r\n\
}\r\n\
\r\n\
.menu-toggle {\r\n\
	background-color: #000;\r\n\
	border-radius: 0;\r\n\
	cursor: pointer;\r\n\
	height: 48px;\r\n\
	margin: 0;\r\n\
	overflow: hidden;\r\n\
	padding: 0;\r\n\
	position: absolute;\r\n\
	top: 0;\r\n\
	right: 0;\r\n\
	text-align: center;\r\n\
	width: 48px;\r\n\
}\r\n\
\r\n\
.menu-toggle:before {\r\n\
	color: #fff;\r\n\
	content: "\f419";\r\n\
	padding: 16px;\r\n\
}\r\n\
\r\n\
.menu-toggle:active,\r\n\
.menu-toggle:focus,\r\n\
.menu-toggle:hover {\r\n\
	background-color: #444;\r\n\
}\r\n\
\r\n\
.menu-toggle:focus {\r\n\
	outline: 1px dotted;\r\n\
}\r\n\
\r\n\
\r\n\
/**\r\n\
 * 6.0 Content\r\n\
 * -----------------------------------------------------------------------------\r\n\
 */\r\n\
\r\n\
.content-area {\r\n\
	padding-top: 48px;\r\n\
}\r\n\
\r\n\
.hentry {\r\n\
	margin: 0 auto 48px;\r\n\
	max-width: 672px;\r\n\
}\r\n\
\r\n\
.site-content .entry-header,\r\n\
.site-content .entry-content,\r\n\
.site-content .entry-summary,\r\n\
.site-content .entry-meta,\r\n\
.page-content {\r\n\
	margin: 0 auto;\r\n\
	max-width: 474px;\r\n\
}\r\n\
\r\n\
.page-content {\r\n\
	margin-bottom: 48px;\r\n\
}\r\n\
\r\n\
\r\n\
/**\r\n\
 * 6.1 Post Thumbnail\r\n\
 * -----------------------------------------------------------------------------\r\n\
 */\r\n\
\r\n\
.post-thumbnail {\r\n\
	background: #b2b2b2 url(images/pattern-light.svg) repeat fixed;\r\n\
	display: block;\r\n\
	position: relative;\r\n\
	width: 100%;\r\n\
	z-index: 0;\r\n\
}\r\n\
\r\n\
a.post-thumbnail:hover {\r\n\
	background-color: #999;\r\n\
}\r\n\
\r\n\
.full-width .post-thumbnail img {\r\n\
	display: block;\r\n\
	margin: 0 auto;\r\n\
}\r\n\
\r\n\
\r\n\
/**\r\n\
 * 6.2 Entry Header\r\n\
 * -----------------------------------------------------------------------------\r\n\
 */\r\n\
\r\n\
.entry-header {\r\n\
	position: relative;\r\n\
	z-index: 1;\r\n\
}\r\n\
\r\n\
.entry-title {\r\n\
	font-size: 33px;\r\n\
	font-weight: 300;\r\n\
	line-height: 1.0909090909;\r\n\
	margin-bottom: 12px;\r\n\
	margin: 0 0 12px 0;\r\n\
	text-transform: none;\r\n\
}\r\n\
\r\n\
.entry-title a {\r\n\
	color: #2b2b2b;\r\n\
}\r\n\
\r\n\
.entry-title a:hover {\r\n\
	color: #41a62a;\r\n\
}\r\n\
\r\n\
.site-content .entry-header {\r\n\
	background-color: #fff;\r\n\
	padding: 0 10px 12px;\r\n\
}\r\n\
\r\n\
.site-content .has-post-thumbnail .entry-header {\r\n\
	padding-top: 24px;\r\n\
}\r\n\
\r\n\
\r\n\
/**\r\n\
 * 6.3 Entry Meta\r\n\
 * -----------------------------------------------------------------------------\r\n\
 */\r\n\
\r\n\
.entry-meta {\r\n\
	clear: both;\r\n\
	color: #767676;\r\n\
	font-size: 12px;\r\n\
	font-weight: 400;\r\n\
	line-height: 1.3333333333;\r\n\
	text-transform: uppercase;\r\n\
}\r\n\
\r\n\
.entry-meta a {\r\n\
	color: #767676;\r\n\
}\r\n\
\r\n\
.entry-meta a:hover {\r\n\
	color: #41a62a;\r\n\
}\r\n\
\r\n\
.sticky .entry-date {\r\n\
	display: none;\r\n\
}\r\n\
\r\n\
.cat-links {\r\n\
	font-weight: 900;\r\n\
	text-transform: uppercase;\r\n\
}\r\n\
\r\n\
.cat-links a {\r\n\
	color: #2b2b2b;\r\n\
}\r\n\
\r\n\
.cat-links a:hover {\r\n\
	color: #41a62a;\r\n\
}\r\n\
\r\n\
.byline {\r\n\
	display: none;\r\n\
}\r\n\
\r\n\
.single .byline,\r\n\
.group-blog .byline {\r\n\
	display: inline;\r\n\
}\r\n\
\r\n\
.site-content .entry-meta {\r\n\
	background-color: #fff;\r\n\
	margin-bottom: 8px;\r\n\
}\r\n\
\r\n\
.site-content footer.entry-meta {\r\n\
	margin: 24px auto 0;\r\n\
	padding: 0 10px;\r\n\
}\r\n\
\r\n\
/* Tag links style */\r\n\
\r\n\
.entry-meta .tag-links a {\r\n\
	background-color: #767676;\r\n\
	border-radius: 0 2px 2px 0;\r\n\
	color: #fff;\r\n\
	display: inline-block;\r\n\
	font-size: 11px;\r\n\
	font-weight: 700;\r\n\
	line-height: 1.2727272727;\r\n\
	margin: 2px 4px 2px 10px;\r\n\
	padding: 3px 7px;\r\n\
	position: relative;\r\n\
	text-transform: uppercase;\r\n\
}\r\n\
\r\n\
.entry-meta .tag-links a:hover {\r\n\
	background-color: #41a62a;\r\n\
	color: #fff;\r\n\
}\r\n\
\r\n\
.entry-meta .tag-links a:before {\r\n\
	border-top: 10px solid transparent;\r\n\
	border-right: 8px solid #767676;\r\n\
	border-bottom: 10px solid transparent;\r\n\
	content: "";\r\n\
	height: 0;\r\n\
	position: absolute;\r\n\
	top: 0;\r\n\
	left: -8px;\r\n\
	width: 0;\r\n\
}\r\n\
\r\n\
.entry-meta .tag-links a:hover:before {\r\n\
	border-right-color: #41a62a;\r\n\
}\r\n\
\r\n\
.entry-meta .tag-links a:after {\r\n\
	background-color: #fff;\r\n\
	border-radius: 50%;\r\n\
	content: "";\r\n\
	height: 4px;\r\n\
	position: absolute;\r\n\
	top: 8px;\r\n\
	left: -2px;\r\n\
	width: 4px;\r\n\
}\r\n\
\r\n\
\r\n\
/**\r\n\
 * 6.4 Entry Content\r\n\
 * -----------------------------------------------------------------------------\r\n\
 */\r\n\
\r\n\
.entry-content,\r\n\
.entry-summary,\r\n\
.page-content {\r\n\
	-webkit-hyphens: auto;\r\n\
	-moz-hyphens:    auto;\r\n\
	-ms-hyphens:     auto;\r\n\
	hyphens:         auto;\r\n\
	word-wrap: break-word;\r\n\
}\r\n\
\r\n\
.site-content .entry-content,\r\n\
.site-content .entry-summary,\r\n\
.page-content {\r\n\
	background-color: #fff;\r\n\
	padding: 12px 10px 0;\r\n\
}\r\n\
\r\n\
.page .entry-content {\r\n\
	padding-top: 0;\r\n\
}\r\n\
\r\n\
.entry-content h1:first-child,\r\n\
.entry-content h2:first-child,\r\n\
.entry-content h3:first-child,\r\n\
.entry-content h4:first-child,\r\n\
.entry-content h5:first-child,\r\n\
.entry-content h6:first-child,\r\n\
.entry-summary h1:first-child,\r\n\
.entry-summary h2:first-child,\r\n\
.entry-summary h3:first-child,\r\n\
.entry-summary h4:first-child,\r\n\
.entry-summary h5:first-child,\r\n\
.entry-summary h6:first-child,\r\n\
.page-content h1:first-child,\r\n\
.page-content h2:first-child,\r\n\
.page-content h3:first-child,\r\n\
.page-content h4:first-child,\r\n\
.page-content h5:first-child,\r\n\
.page-content h6:first-child {\r\n\
	margin-top: 0;\r\n\
}\r\n\
\r\n\
.entry-content a,\r\n\
.entry-summary a,\r\n\
.page-content a,\r\n\
.comment-content a {\r\n\
	text-decoration: underline;\r\n\
}\r\n\
\r\n\
.entry-content a:hover,\r\n\
.entry-summary a:hover,\r\n\
.page-content a:hover,\r\n\
.comment-content a:hover,\r\n\
.entry-content a.button,\r\n\
.entry-summary a.button,\r\n\
.page-content a.button,\r\n\
.comment-content a.button {\r\n\
	text-decoration: none;\r\n\
}\r\n\
\r\n\
.entry-content table,\r\n\
.comment-content table {\r\n\
	font-size: 14px;\r\n\
	line-height: 1.2857142857;\r\n\
	margin-bottom: 24px;\r\n\
}\r\n\
\r\n\
.entry-content th,\r\n\
.comment-content th {\r\n\
	font-weight: 700;\r\n\
	padding: 8px;\r\n\
	text-transform: uppercase;\r\n\
}\r\n\
\r\n\
.entry-content td,\r\n\
.comment-content td {\r\n\
	padding: 8px;\r\n\
}\r\n\
\r\n\
.entry-content .edit-link {\r\n\
	clear: both;\r\n\
	display: block;\r\n\
	font-size: 12px;\r\n\
	font-weight: 400;\r\n\
	line-height: 1.3333333333;\r\n\
	text-transform: uppercase;\r\n\
}\r\n\
\r\n\
.entry-content .edit-link a {\r\n\
	color: #767676;\r\n\
	text-decoration: none;\r\n\
}\r\n\
\r\n\
.entry-content .edit-link a:hover {\r\n\
	color: #41a62a;\r\n\
}\r\n\
\r\n\
.entry-content .more-link {\r\n\
	white-space: nowrap;\r\n\
}\r\n\
\r\n\
/* Mediaelements */\r\n\
\r\n\
.hentry .mejs-container {\r\n\
	margin: 12px 0 18px;\r\n\
}\r\n\
\r\n\
.hentry .mejs-mediaelement,\r\n\
.hentry .mejs-container .mejs-controls {\r\n\
	background: #000;\r\n\
}\r\n\
\r\n\
.hentry .mejs-controls .mejs-time-rail .mejs-time-loaded,\r\n\
.hentry .mejs-controls .mejs-horizontal-volume-slider .mejs-horizontal-volume-current {\r\n\
	background: #fff;\r\n\
}\r\n\
\r\n\
.hentry .mejs-controls .mejs-time-rail .mejs-time-current {\r\n\
	background: #24890d;\r\n\
}\r\n\
\r\n\
.hentry .mejs-controls .mejs-time-rail .mejs-time-total,\r\n\
.hentry .mejs-controls .mejs-horizontal-volume-slider .mejs-horizontal-volume-total {\r\n\
	background: rgba(255, 255, 255, .33);\r\n\
}\r\n\
\r\n\
.hentry .mejs-container .mejs-controls .mejs-time {\r\n\
	padding-top: 9px;\r\n\
}\r\n\
\r\n\
.hentry .mejs-controls .mejs-time-rail span,\r\n\
.hentry .mejs-controls .mejs-horizontal-volume-slider .mejs-horizontal-volume-total,\r\n\
.hentry .mejs-controls .mejs-horizontal-volume-slider .mejs-horizontal-volume-current {\r\n\
	border-radius: 0;\r\n\
}\r\n\
\r\n\
.hentry .mejs-overlay-loading {\r\n\
	background: transparent;\r\n\
}\r\n\
\r\n\
.hentry .mejs-overlay-button {\r\n\
	background-color: #fff;\r\n\
	background-image: none;\r\n\
	border-radius: 2px;\r\n\
	box-shadow: 1px 1px 1px rgba(0,0,0,.8);\r\n\
	color: #000;\r\n\
	height: 36px;\r\n\
	margin-left: -24px;\r\n\
	width: 48px;\r\n\
}\r\n\
\r\n\
.hentry .mejs-overlay-button:before {\r\n\
	-webkit-font-smoothing: antialiased;\r\n\
	content: \'\f452\';\r\n\
	display: inline-block;\r\n\
	font: normal 32px/1.125 Genericons;\r\n\
	position: absolute;\r\n\
	top: 1px;\r\n\
	left: 10px;\r\n\
}\r\n\
\r\n\
.hentry .mejs-controls .mejs-button button:focus {\r\n\
	outline: none;\r\n\
}\r\n\
\r\n\
.hentry .mejs-controls .mejs-button button {\r\n\
	-webkit-font-smoothing: antialiased;\r\n\
	background: none;\r\n\
	color: #fff;\r\n\
	display: inline-block;\r\n\
	font: normal 16px/1 Genericons;\r\n\
}\r\n\
\r\n\
.hentry .mejs-playpause-button.mejs-play button:before {\r\n\
	content: \'\f452\';\r\n\
}\r\n\
\r\n\
.hentry .mejs-playpause-button.mejs-pause button:before {\r\n\
	content: \'\f448\';\r\n\
}\r\n\
\r\n\
.hentry .mejs-volume-button.mejs-mute button:before {\r\n\
	content: \'\f109\';\r\n\
	font-size: 20px;\r\n\
	position: absolute;\r\n\
	top: -2px;\r\n\
	left: 0;\r\n\
}\r\n\
\r\n\
.hentry .mejs-volume-button.mejs-unmute button:before {\r\n\
	content: \'\f109\';\r\n\
	left: 0;\r\n\
	position: absolute;\r\n\
	top: 0;\r\n\
}\r\n\
\r\n\
.hentry .mejs-fullscreen-button button:before {\r\n\
	content: \'\f474\';\r\n\
}\r\n\
\r\n\
.hentry .mejs-fullscreen-button.mejs-unfullscreen button:before {\r\n\
	content: \'\f406\';\r\n\
}\r\n\
\r\n\
.hentry .mejs-overlay:hover .mejs-overlay-button {\r\n\
	background-color: #24890d;\r\n\
	color: #fff;\r\n\
}\r\n\
\r\n\
.hentry .mejs-controls .mejs-button button:hover {\r\n\
	color: #41a62a;\r\n\
}\r\n\
\r\n\
.content-sidebar .wp-playlist-item .wp-playlist-caption {\r\n\
	color: #000;\r\n\
}\r\n\
\r\n\
/* Page links */\r\n\
\r\n\
.page-links {\r\n\
	clear: both;\r\n\
	font-size: 12px;\r\n\
	font-weight: 900;\r\n\
	line-height: 2;\r\n\
	margin: 24px 0;\r\n\
	text-transform: uppercase;\r\n\
}\r\n\
\r\n\
.page-links a,\r\n\
.page-links > span {\r\n\
	background: #fff;\r\n\
	border: 1px solid #fff;\r\n\
	display: inline-block;\r\n\
	height: 22px;\r\n\
	margin: 0 1px 2px 0;\r\n\
	text-align: center;\r\n\
	width: 22px;\r\n\
}\r\n\
\r\n\
.page-links a {\r\n\
	background: #000;\r\n\
	border: 1px solid #000;\r\n\
	color: #fff;\r\n\
	text-decoration: none;\r\n\
}\r\n\
\r\n\
.page-links a:hover {\r\n\
	background: #41a62a;\r\n\
	border: 1px solid #41a62a;\r\n\
	color: #fff;\r\n\
}\r\n\
\r\n\
.page-links > .page-links-title {\r\n\
	height: auto;\r\n\
	margin: 0;\r\n\
	padding-right: 7px;\r\n\
	width: auto;\r\n\
}\r\n\
\r\n\
\r\n\
/**\r\n\
 * 6.5 Gallery\r\n\
 * -----------------------------------------------------------------------------\r\n\
 */\r\n\
\r\n\
.gallery {\r\n\
	margin-bottom: 20px;\r\n\
}\r\n\
\r\n\
.gallery-item {\r\n\
	float: left;\r\n\
	margin: 0 4px 4px 0;\r\n\
	overflow: hidden;\r\n\
	position: relative;\r\n\
}\r\n\
\r\n\
.gallery-columns-1 .gallery-item {\r\n\
	max-width: 100%;\r\n\
}\r\n\
\r\n\
.gallery-columns-2 .gallery-item {\r\n\
	max-width: 48%;\r\n\
	max-width: -webkit-calc(50% - 4px);\r\n\
	max-width:         calc(50% - 4px);\r\n\
}\r\n\
\r\n\
.gallery-columns-3 .gallery-item {\r\n\
	max-width: 32%;\r\n\
	max-width: -webkit-calc(33.3% - 4px);\r\n\
	max-width:         calc(33.3% - 4px);\r\n\
}\r\n\
\r\n\
.gallery-columns-4 .gallery-item {\r\n\
	max-width: 23%;\r\n\
	max-width: -webkit-calc(25% - 4px);\r\n\
	max-width:         calc(25% - 4px);\r\n\
}\r\n\
\r\n\
.gallery-columns-5 .gallery-item {\r\n\
	max-width: 19%;\r\n\
	max-width: -webkit-calc(20% - 4px);\r\n\
	max-width:         calc(20% - 4px);\r\n\
}\r\n\
\r\n\
.gallery-columns-6 .gallery-item {\r\n\
	max-width: 15%;\r\n\
	max-width: -webkit-calc(16.7% - 4px);\r\n\
	max-width:         calc(16.7% - 4px);\r\n\
}\r\n\
\r\n\
.gallery-columns-7 .gallery-item {\r\n\
	max-width: 13%;\r\n\
	max-width: -webkit-calc(14.28% - 4px);\r\n\
	max-width:         calc(14.28% - 4px);\r\n\
}\r\n\
\r\n\
.gallery-columns-8 .gallery-item {\r\n\
	max-width: 11%;\r\n\
	max-width: -webkit-calc(12.5% - 4px);\r\n\
	max-width:         calc(12.5% - 4px);\r\n\
}\r\n\
\r\n\
.gallery-columns-9 .gallery-item {\r\n\
	max-width: 9%;\r\n\
	max-width: -webkit-calc(11.1% - 4px);\r\n\
	max-width:         calc(11.1% - 4px);\r\n\
}\r\n\
\r\n\
.gallery-columns-1 .gallery-item:nth-of-type(1n),\r\n\
.gallery-columns-2 .gallery-item:nth-of-type(2n),\r\n\
.gallery-columns-3 .gallery-item:nth-of-type(3n),\r\n\
.gallery-columns-4 .gallery-item:nth-of-type(4n),\r\n\
.gallery-columns-5 .gallery-item:nth-of-type(5n),\r\n\
.gallery-columns-6 .gallery-item:nth-of-type(6n),\r\n\
.gallery-columns-7 .gallery-item:nth-of-type(7n),\r\n\
.gallery-columns-8 .gallery-item:nth-of-type(8n),\r\n\
.gallery-columns-9 .gallery-item:nth-of-type(9n) {\r\n\
	margin-right: 0;\r\n\
}\r\n\
\r\n\
.gallery-columns-1.gallery-size-medium figure.gallery-item:nth-of-type(1n+1),\r\n\
.gallery-columns-1.gallery-size-thumbnail figure.gallery-item:nth-of-type(1n+1),\r\n\
.gallery-columns-2.gallery-size-thumbnail figure.gallery-item:nth-of-type(2n+1),\r\n\
.gallery-columns-3.gallery-size-thumbnail figure.gallery-item:nth-of-type(3n+1) {\r\n\
	clear: left;\r\n\
}\r\n\
\r\n\
.gallery-caption {\r\n\
	background-color: rgba(0, 0, 0, 0.7);\r\n\
	-webkit-box-sizing: border-box;\r\n\
	-moz-box-sizing:    border-box;\r\n\
	box-sizing:         border-box;\r\n\
	color: #fff;\r\n\
	font-size: 12px;\r\n\
	line-height: 1.5;\r\n\
	margin: 0;\r\n\
	max-height: 50%;\r\n\
	opacity: 0;\r\n\
	padding: 6px 8px;\r\n\
	position: absolute;\r\n\
	bottom: 0;\r\n\
	left: 0;\r\n\
	text-align: left;\r\n\
	width: 100%;\r\n\
}\r\n\
\r\n\
.gallery-caption:before {\r\n\
	content: "";\r\n\
	height: 100%;\r\n\
	min-height: 49px;\r\n\
	position: absolute;\r\n\
	top: 0;\r\n\
	left: 0;\r\n\
	width: 100%;\r\n\
}\r\n\
\r\n\
.gallery-item:hover .gallery-caption {\r\n\
	opacity: 1;\r\n\
}\r\n\
\r\n\
.gallery-columns-7 .gallery-caption,\r\n\
.gallery-columns-8 .gallery-caption,\r\n\
.gallery-columns-9 .gallery-caption {\r\n\
	display: none;\r\n\
}\r\n\
\r\n\
\r\n\
/**\r\n\
 * 6.6 Post Formats\r\n\
 * -----------------------------------------------------------------------------\r\n\
 */\r\n\
\r\n\
.format-aside .entry-content,\r\n\
.format-aside .entry-summary,\r\n\
.format-quote .entry-content,\r\n\
.format-quote .entry-summary,\r\n\
.format-link .entry-content,\r\n\
.format-link .entry-summary {\r\n\
	padding-top: 0;\r\n\
}\r\n\
\r\n\
.site-content .format-link .entry-title,\r\n\
.site-content .format-aside .entry-title,\r\n\
.site-content .format-quote .entry-title {\r\n\
	display: none;\r\n\
}\r\n\
\r\n\
\r\n\
/**\r\n\
 * 6.7 Post/Image/Paging Navigation\r\n\
 * -----------------------------------------------------------------------------\r\n\
 */\r\n\
\r\n\
.nav-links {\r\n\
	-webkit-hyphens: auto;\r\n\
	-moz-hyphens:    auto;\r\n\
	-ms-hyphens:     auto;\r\n\
	border-top: 1px solid rgba(0, 0, 0, 0.1);\r\n\
	hyphens:         auto;\r\n\
	word-wrap: break-word;\r\n\
}\r\n\
\r\n\
.post-navigation,\r\n\
.image-navigation {\r\n\
	margin: 24px auto 48px;\r\n\
	max-width: 474px;\r\n\
	padding: 0 10px;\r\n\
}\r\n\
\r\n\
.post-navigation a,\r\n\
.image-navigation .previous-image,\r\n\
.image-navigation .next-image {\r\n\
	border-bottom: 1px solid rgba(0, 0, 0, 0.1);\r\n\
	padding: 11px 0 12px;\r\n\
	width: 100%;\r\n\
}\r\n\
\r\n\
.post-navigation .meta-nav {\r\n\
	color: #767676;\r\n\
	display: block;\r\n\
	font-size: 12px;\r\n\
	font-weight: 900;\r\n\
	line-height: 2;\r\n\
	text-transform: uppercase;\r\n\
}\r\n\
\r\n\
.post-navigation a,\r\n\
.image-navigation a {\r\n\
	color: #2b2b2b;\r\n\
	display: block;\r\n\
	font-size: 14px;\r\n\
	font-weight: 700;\r\n\
	line-height: 1.7142857142;\r\n\
	text-transform: none;\r\n\
}\r\n\
\r\n\
.post-navigation a:hover,\r\n\
.image-navigation a:hover {\r\n\
	color: #41a62a;\r\n\
}\r\n\
\r\n\
/* Paging Navigation */\r\n\
\r\n\
.paging-navigation {\r\n\
	border-top: 5px solid #000;\r\n\
	margin: 48px 0;\r\n\
}\r\n\
\r\n\
.paging-navigation .loop-pagination {\r\n\
	margin-top: -5px;\r\n\
	text-align: center;\r\n\
}\r\n\
\r\n\
.paging-navigation .page-numbers {\r\n\
	border-top: 5px solid transparent;\r\n\
	display: inline-block;\r\n\
	font-size: 14px;\r\n\
	font-weight: 900;\r\n\
	margin-right: 1px;\r\n\
	padding: 7px 16px;\r\n\
	text-transform: uppercase;\r\n\
}\r\n\
\r\n\
.paging-navigation a {\r\n\
	color: #2b2b2b;\r\n\
}\r\n\
\r\n\
.paging-navigation .page-numbers.current {\r\n\
	border-top: 5px solid #24890d;\r\n\
}\r\n\
\r\n\
.paging-navigation a:hover {\r\n\
	border-top: 5px solid #41a62a;\r\n\
	color: #2b2b2b;\r\n\
}\r\n\
\r\n\
\r\n\
/**\r\n\
 * 6.8 Attachments\r\n\
 * -----------------------------------------------------------------------------\r\n\
 */\r\n\
\r\n\
.attachment .content-sidebar,\r\n\
.attachment .post-thumbnail {\r\n\
	display: none;\r\n\
}\r\n\
\r\n\
.attachment .entry-content {\r\n\
	padding-top: 0;\r\n\
}\r\n\
\r\n\
.attachment footer.entry-meta {\r\n\
	text-transform: none;\r\n\
}\r\n\
\r\n\
.entry-attachment .attachment {\r\n\
	margin-bottom: 24px;\r\n\
}\r\n\
\r\n\
\r\n\
/**\r\n\
 * 6.9 Archives\r\n\
 * -----------------------------------------------------------------------------\r\n\
 */\r\n\
\r\n\
.archive-header,\r\n\
.page-header {\r\n\
	margin: 24px auto;\r\n\
	max-width: 474px;\r\n\
}\r\n\
\r\n\
.archive-title,\r\n\
.page-title {\r\n\
	font-size: 16px;\r\n\
	font-weight: 900;\r\n\
	line-height: 1.5;\r\n\
	margin: 0;\r\n\
}\r\n\
\r\n\
.taxonomy-description,\r\n\
.author-description {\r\n\
	color: #767676;\r\n\
	font-size: 14px;\r\n\
	line-height: 1.2857142857;\r\n\
	padding-top: 18px;\r\n\
}\r\n\
\r\n\
.taxonomy-description p,\r\n\
.author-description p {\r\n\
	margin-bottom: 18px;\r\n\
}\r\n\
\r\n\
.taxonomy-description p:last-child,\r\n\
.author-description p:last-child {\r\n\
	margin-bottom: 0;\r\n\
}\r\n\
\r\n\
.taxonomy-description a,\r\n\
.author-description a {\r\n\
	text-decoration: underline;\r\n\
}\r\n\
\r\n\
.taxonomy-description a:hover,\r\n\
.author-description a:hover {\r\n\
	text-decoration: none;\r\n\
}\r\n\
\r\n\
\r\n\
/**\r\n\
 * 6.10 Contributor Page\r\n\
 * -----------------------------------------------------------------------------\r\n\
 */\r\n\
\r\n\
.contributor {\r\n\
	border-bottom: 1px solid rgba(0, 0, 0, 0.1);\r\n\
	-webkit-box-sizing: border-box;\r\n\
	-moz-box-sizing: border-box;\r\n\
	box-sizing:      border-box;\r\n\
	padding: 48px 10px;\r\n\
}\r\n\
\r\n\
.contributor:first-of-type {\r\n\
	padding-top: 24px;\r\n\
}\r\n\
\r\n\
.contributor-info {\r\n\
	margin: 0 auto;\r\n\
	max-width: 474px;\r\n\
}\r\n\
\r\n\
.contributor-avatar {\r\n\
	border: 1px solid rgba(0, 0, 0, 0.1);\r\n\
	float: left;\r\n\
	margin: 0 30px 20px 0;\r\n\
	padding: 2px;\r\n\
}\r\n\
\r\n\
.contributor-name {\r\n\
	font-size: 16px;\r\n\
	font-weight: 900;\r\n\
	line-height: 1.5;\r\n\
	margin: 0;\r\n\
}\r\n\
\r\n\
.contributor-bio a {\r\n\
	text-decoration: underline;\r\n\
}\r\n\
\r\n\
.contributor-bio a:hover {\r\n\
	text-decoration: none;\r\n\
}\r\n\
\r\n\
.contributor-posts-link {\r\n\
	display: inline-block;\r\n\
	line-height: normal;\r\n\
	padding: 10px 30px;\r\n\
}\r\n\
\r\n\
.contributor-posts-link:before {\r\n\
	content: "\f443";\r\n\
}\r\n\
\r\n\
\r\n\
/**\r\n\
 * 6.11 404 Page\r\n\
 * -----------------------------------------------------------------------------\r\n\
 */\r\n\
\r\n\
.error404 .page-content {\r\n\
	padding-top: 0;\r\n\
}\r\n\
\r\n\
.error404 .page-content .search-form {\r\n\
	margin-bottom: 24px;\r\n\
}\r\n\
\r\n\
\r\n\
/**\r\n\
 * 6.12 Full-width\r\n\
 * -----------------------------------------------------------------------------\r\n\
 */\r\n\
\r\n\
.full-width .hentry {\r\n\
	max-width: 100%;\r\n\
}\r\n\
\r\n\
\r\n\
/**\r\n\
 * 6.13 Singular\r\n\
 * -----------------------------------------------------------------------------\r\n\
 */\r\n\
\r\n\
.singular .site-content .hentry.has-post-thumbnail {\r\n\
	margin-top: -48px;\r\n\
}\r\n\
\r\n\
\r\n\
/**\r\n\
 * 6.14 Comments\r\n\
 * -----------------------------------------------------------------------------\r\n\
 */\r\n\
\r\n\
.comments-area {\r\n\
	margin: 48px auto;\r\n\
	max-width: 474px;\r\n\
	padding: 0 10px;\r\n\
}\r\n\
\r\n\
.comment-reply-title,\r\n\
.comments-title {\r\n\
	font: 900 16px/1.5 Lato, sans-serif;\r\n\
	margin: 0;\r\n\
	text-transform: uppercase;\r\n\
}\r\n\
\r\n\
.comment-list {\r\n\
	list-style: none;\r\n\
	margin: 0 0 48px 0;\r\n\
}\r\n\
\r\n\
.comment-author {\r\n\
	font-size: 14px;\r\n\
	line-height: 1.7142857142;\r\n\
}\r\n\
\r\n\
.comment-list .reply,\r\n\
.comment-metadata {\r\n\
	font-size: 12px;\r\n\
	line-height: 2;\r\n\
	text-transform: uppercase;\r\n\
}\r\n\
\r\n\
.comment-list .reply {\r\n\
	margin-top: 24px;\r\n\
}\r\n\
\r\n\
.comment-author .fn {\r\n\
	font-weight: 900;\r\n\
}\r\n\
\r\n\
.comment-author a {\r\n\
	color: #2b2b2b;\r\n\
}\r\n\
\r\n\
.comment-list .trackback a,\r\n\
.comment-list .pingback a,\r\n\
.comment-metadata a {\r\n\
	color: #767676;\r\n\
}\r\n\
\r\n\
.comment-author a:hover,\r\n\
.comment-list .pingback a:hover,\r\n\
.comment-list .trackback a:hover,\r\n\
.comment-metadata a:hover {\r\n\
	color: #41a62a;\r\n\
}\r\n\
\r\n\
.comment-list article,\r\n\
.comment-list .pingback,\r\n\
.comment-list .trackback {\r\n\
	border-top: 1px solid rgba(0, 0, 0, 0.1);\r\n\
	margin-bottom: 24px;\r\n\
	padding-top: 24px;\r\n\
}\r\n\
\r\n\
.comment-list > li:first-child > article,\r\n\
.comment-list > .pingback:first-child,\r\n\
.comment-list > .trackback:first-child {\r\n\
	border-top: 0;\r\n\
}\r\n\
\r\n\
.comment-author {\r\n\
	position: relative;\r\n\
}\r\n\
\r\n\
.comment-author .avatar {\r\n\
	border: 1px solid rgba(0, 0, 0, 0.1);\r\n\
	height: 18px;\r\n\
	padding: 2px;\r\n\
	position: absolute;\r\n\
	top: 0;\r\n\
	left: 0;\r\n\
	width: 18px;\r\n\
}\r\n\
\r\n\
.bypostauthor > article .fn:before {\r\n\
	content: "\f408";\r\n\
	margin: 0 2px 0 -2px;\r\n\
	position: relative;\r\n\
	top: -1px;\r\n\
}\r\n\
\r\n\
.says {\r\n\
	display: none;\r\n\
}\r\n\
\r\n\
.comment-author,\r\n\
.comment-awaiting-moderation,\r\n\
.comment-content,\r\n\
.comment-list .reply,\r\n\
.comment-metadata {\r\n\
	padding-left: 30px;\r\n\
}\r\n\
\r\n\
.comment-edit-link {\r\n\
	margin-left: 10px;\r\n\
}\r\n\
\r\n\
.comment-edit-link:before {\r\n\
	content: "\f411";\r\n\
}\r\n\
\r\n\
.comment-reply-link:before,\r\n\
.comment-reply-login:before {\r\n\
	content: "\f412";\r\n\
	margin-right: 2px;\r\n\
}\r\n\
\r\n\
.comment-content {\r\n\
	-webkit-hyphens: auto;\r\n\
	-moz-hyphens:    auto;\r\n\
	-ms-hyphens:     auto;\r\n\
	hyphens:         auto;\r\n\
	word-wrap: break-word;\r\n\
}\r\n\
\r\n\
.comment-content ul,\r\n\
.comment-content ol {\r\n\
	margin: 0 0 24px 22px;\r\n\
}\r\n\
\r\n\
.comment-content li > ul,\r\n\
.comment-content li > ol {\r\n\
	margin-bottom: 0;\r\n\
}\r\n\
\r\n\
.comment-content > :last-child {\r\n\
	margin-bottom: 0;\r\n\
}\r\n\
\r\n\
.comment-list .children {\r\n\
	list-style: none;\r\n\
	margin-left: 15px;\r\n\
}\r\n\
\r\n\
.comment-respond {\r\n\
	margin-bottom: 24px;\r\n\
	padding: 0;\r\n\
}\r\n\
\r\n\
.comment .comment-respond {\r\n\
	margin-top: 24px;\r\n\
}\r\n\
\r\n\
.comment-respond h3 {\r\n\
	margin-top: 0;\r\n\
	margin-bottom: 24px;\r\n\
}\r\n\
\r\n\
.comment-notes,\r\n\
.comment-awaiting-moderation,\r\n\
.logged-in-as,\r\n\
.no-comments,\r\n\
.form-allowed-tags,\r\n\
.form-allowed-tags code {\r\n\
	color: #767676;\r\n\
}\r\n\
\r\n\
.comment-notes,\r\n\
.comment-awaiting-moderation,\r\n\
.logged-in-as {\r\n\
	font-size: 14px;\r\n\
	line-height: 1.7142857142;\r\n\
}\r\n\
\r\n\
.no-comments {\r\n\
	font-size: 16px;\r\n\
	font-weight: 900;\r\n\
	line-height: 1.5;\r\n\
	margin-top: 24px;\r\n\
	text-transform: uppercase;\r\n\
}\r\n\
\r\n\
.comment-form label {\r\n\
	display: block;\r\n\
}\r\n\
\r\n\
.comment-form input[type="text"],\r\n\
.comment-form input[type="email"],\r\n\
.comment-form input[type="url"] {\r\n\
	width: 100%;\r\n\
}\r\n\
\r\n\
.form-allowed-tags,\r\n\
.form-allowed-tags code {\r\n\
	font-size: 12px;\r\n\
	line-height: 1.5;\r\n\
}\r\n\
\r\n\
.required {\r\n\
	color: #c0392b;\r\n\
}\r\n\
\r\n\
.comment-reply-title small a {\r\n\
	color: #2b2b2b;\r\n\
	float: right;\r\n\
	height: 24px;\r\n\
	overflow: hidden;\r\n\
	width: 24px;\r\n\
}\r\n\
\r\n\
.comment-reply-title small a:hover {\r\n\
	color: #41a62a;\r\n\
}\r\n\
\r\n\
.comment-reply-title small a:before {\r\n\
	content: "\f405";\r\n\
	font-size: 32px;\r\n\
}\r\n\
\r\n\
.comment-navigation {\r\n\
	font-size: 12px;\r\n\
	line-height: 2;\r\n\
	margin-bottom: 48px;\r\n\
	text-transform: uppercase;\r\n\
}\r\n\
\r\n\
.comment-navigation .nav-next,\r\n\
.comment-navigation .nav-previous {\r\n\
	display: inline-block;\r\n\
}\r\n\
\r\n\
.comment-navigation .nav-previous a {\r\n\
	margin-right: 10px;\r\n\
}\r\n\
\r\n\
#comment-nav-above {\r\n\
	margin-top: 36px;\r\n\
	margin-bottom: 0;\r\n\
}\r\n\
\r\n\
\r\n\
/**\r\n\
 * 7.0 Sidebars\r\n\
 * -----------------------------------------------------------------------------\r\n\
 */\r\n\
\r\n\
/* Secondary */\r\n\
\r\n\
#secondary {\r\n\
	background-color: #000;\r\n\
	border-top: 1px solid #000;\r\n\
	border-bottom: 1px solid rgba(255, 255, 255, 0.2);\r\n\
	clear: both;\r\n\
	color: rgba(255, 255, 255, 0.7);\r\n\
	margin-top: -1px;\r\n\
	padding: 0 10px;\r\n\
	position: relative;\r\n\
	z-index: 2;\r\n\
}\r\n\
\r\n\
.site-description {\r\n\
	display: none;\r\n\
	font-size: 12px;\r\n\
	font-weight: 400;\r\n\
	line-height: 1.5;\r\n\
}\r\n\
\r\n\
/* Primary Sidebar */\r\n\
\r\n\
.primary-sidebar {\r\n\
	padding-top: 48px;\r\n\
}\r\n\
\r\n\
.secondary-navigation + .primary-sidebar {\r\n\
	padding-top: 0;\r\n\
}\r\n\
\r\n\
/* Content Sidebar */\r\n\
\r\n\
.content-sidebar {\r\n\
	border-top: 1px solid rgba(0, 0, 0, 0.1);\r\n\
	border-bottom: 1px solid rgba(0, 0, 0, 0.1);\r\n\
	-webkit-box-sizing: border-box;\r\n\
	-moz-box-sizing:    border-box;\r\n\
	box-sizing:         border-box;\r\n\
	color: #767676;\r\n\
	padding: 48px 10px 0;\r\n\
}\r\n\
\r\n\
\r\n\
/**\r\n\
 * 7.1 Widgets\r\n\
 * -----------------------------------------------------------------------------\r\n\
 */\r\n\
\r\n\
/* Primary Sidebar, Footer Sidebar */\r\n\
\r\n\
.widget {\r\n\
	font-size: 14px;\r\n\
	-webkit-hyphens: auto;\r\n\
	-moz-hyphens:    auto;\r\n\
	-ms-hyphens:     auto;\r\n\
	hyphens:         auto;\r\n\
	line-height: 1.2857142857;\r\n\
	margin-bottom: 48px;\r\n\
	width: 100%;\r\n\
	word-wrap: break-word;\r\n\
}\r\n\
\r\n\
.widget a {\r\n\
	color: #fff;\r\n\
}\r\n\
\r\n\
.widget a:hover {\r\n\
	color: #41a62a;\r\n\
}\r\n\
\r\n\
.widget h1,\r\n\
.widget h2,\r\n\
.widget h3,\r\n\
.widget h4,\r\n\
.widget h5,\r\n\
.widget h6 {\r\n\
	margin: 24px 0 12px;\r\n\
}\r\n\
\r\n\
.widget h1 {\r\n\
	font-size: 22px;\r\n\
	line-height: 1.0909090909;\r\n\
}\r\n\
\r\n\
.widget h2 {\r\n\
	font-size: 20px;\r\n\
	line-height: 1.2;\r\n\
}\r\n\
\r\n\
.widget h3 {\r\n\
	font-size: 18px;\r\n\
	line-height: 1.3333333333;\r\n\
}\r\n\
\r\n\
.widget h4 {\r\n\
	font-size: 16px;\r\n\
	line-height: 1.5;\r\n\
}\r\n\
\r\n\
.widget h5 {\r\n\
	font-size: 14px;\r\n\
	line-height: 1.7142857142;\r\n\
}\r\n\
\r\n\
.widget h6 {\r\n\
	font-size: 12px;\r\n\
	line-height: 2;\r\n\
}\r\n\
\r\n\
.widget address {\r\n\
	margin-bottom: 18px;\r\n\
}\r\n\
\r\n\
.widget abbr[title] {\r\n\
	border-color: rgba(255, 255, 255, 0.7);\r\n\
}\r\n\
\r\n\
.widget mark,\r\n\
.widget ins {\r\n\
	color: #000;\r\n\
}\r\n\
\r\n\
.widget pre,\r\n\
.widget fieldset {\r\n\
	border-color: rgba(255, 255, 255, 0.2);\r\n\
}\r\n\
\r\n\
.widget code,\r\n\
.widget kbd,\r\n\
.widget tt,\r\n\
.widget var,\r\n\
.widget samp,\r\n\
.widget pre {\r\n\
	font-size: 12px;\r\n\
	line-height: 1.5;\r\n\
}\r\n\
\r\n\
.widget blockquote {\r\n\
	color: rgba(255, 255, 255, 0.7);\r\n\
	font-size: 18px;\r\n\
	line-height: 1.5;\r\n\
	margin-bottom: 18px;\r\n\
}\r\n\
\r\n\
.widget blockquote cite {\r\n\
	color: #fff;\r\n\
	font-size: 14px;\r\n\
	line-height: 1.2857142857;\r\n\
}\r\n\
\r\n\
.widget dl,\r\n\
.widget dd {\r\n\
	margin-bottom: 18px;\r\n\
}\r\n\
\r\n\
.widget ul,\r\n\
.widget ol {\r\n\
	list-style: none;\r\n\
	margin: 0;\r\n\
}\r\n\
\r\n\
.widget li > ol,\r\n\
.widget li > ul {\r\n\
	margin-left: 10px;\r\n\
}\r\n\
\r\n\
.widget table,\r\n\
.widget th,\r\n\
.widget td {\r\n\
	border-color: rgba(255, 255, 255, 0.2);\r\n\
}\r\n\
\r\n\
.widget table {\r\n\
	margin-bottom: 18px;\r\n\
}\r\n\
\r\n\
.widget del {\r\n\
	color: rgba(255, 255, 255, 0.4);\r\n\
}\r\n\
\r\n\
.widget hr {\r\n\
	background-color: rgba(255, 255, 255, 0.2);\r\n\
}\r\n\
\r\n\
.widget p {\r\n\
	margin-bottom: 18px;\r\n\
}\r\n\
\r\n\
.widget-area .widget input,\r\n\
.widget-area .widget textarea {\r\n\
	background-color: rgba(255, 255, 255, 0.1);\r\n\
	border-color: rgba(255, 255, 255, 0.2);\r\n\
	color: #fff;\r\n\
	font-size: 16px;\r\n\
	padding: 1px 2px 2px 4px;\r\n\
}\r\n\
\r\n\
.widget-area .widget input:focus,\r\n\
.widget-area .widget textarea:focus {\r\n\
	border-color: rgba(255, 255, 255, 0.3);\r\n\
}\r\n\
\r\n\
.widget button,\r\n\
.widget .button,\r\n\
.widget input[type="button"],\r\n\
.widget input[type="reset"],\r\n\
.widget input[type="submit"] {\r\n\
	background-color: #24890d;\r\n\
	border: 0;\r\n\
	font-size: 12px;\r\n\
	padding: 5px 15px 4px;\r\n\
}\r\n\
\r\n\
.widget input[type="button"]:hover,\r\n\
.widget input[type="button"]:focus,\r\n\
.widget input[type="reset"]:hover,\r\n\
.widget input[type="reset"]:focus,\r\n\
.widget input[type="submit"]:hover,\r\n\
.widget input[type="submit"]:focus {\r\n\
	background-color: #41a62a;\r\n\
}\r\n\
\r\n\
.widget input[type="button"]:active,\r\n\
.widget input[type="reset"]:active,\r\n\
.widget input[type="submit"]:active {\r\n\
	background-color: #55d737;\r\n\
}\r\n\
\r\n\
.widget .wp-caption {\r\n\
	color: rgba(255, 255, 255, 0.7);\r\n\
	margin-bottom: 18px;\r\n\
}\r\n\
\r\n\
.widget .widget-title {\r\n\
	font-size: 14px;\r\n\
	font-weight: 700;\r\n\
	line-height: 1.7142857142;\r\n\
	margin: 0 0 24px 0;\r\n\
	text-transform: uppercase;\r\n\
}\r\n\
\r\n\
.widget-title,\r\n\
.widget-title a {\r\n\
	color: #fff;\r\n\
}\r\n\
\r\n\
.widget-title a:hover {\r\n\
	color: #41a62a;\r\n\
}\r\n\
\r\n\
/* Calendar Widget*/\r\n\
\r\n\
.widget_calendar table {\r\n\
	line-height: 2;\r\n\
	margin: 0;\r\n\
}\r\n\
\r\n\
.widget_calendar caption {\r\n\
	color: #fff;\r\n\
	font-weight: 700;\r\n\
	line-height: 1.7142857142;\r\n\
	margin-bottom: 18px;\r\n\
	text-align: left;\r\n\
	text-transform: uppercase;\r\n\
}\r\n\
\r\n\
.widget_calendar thead th {\r\n\
	background-color: rgba(255, 255, 255, 0.1);\r\n\
}\r\n\
\r\n\
.widget_calendar tbody td,\r\n\
.widget_calendar thead th {\r\n\
	text-align: center;\r\n\
}\r\n\
\r\n\
.widget_calendar tbody a {\r\n\
	background-color: #24890d;\r\n\
	color: #fff;\r\n\
	display: block;\r\n\
}\r\n\
\r\n\
.widget_calendar tbody a:hover {\r\n\
	background-color: #41a62a;\r\n\
}\r\n\
\r\n\
.widget_calendar tbody a:hover {\r\n\
	color: #fff;\r\n\
}\r\n\
\r\n\
.widget_calendar #prev {\r\n\
	padding-left: 5px;\r\n\
}\r\n\
\r\n\
.widget_calendar #next {\r\n\
	padding-right: 5px;\r\n\
	text-align: right;\r\n\
}\r\n\
\r\n\
/* Ephemera Widget*/\r\n\
\r\n\
.widget_twentyfourteen_ephemera > ol > li {\r\n\
	border-bottom: 1px solid rgba(255, 255, 255, 0.2);\r\n\
	margin-bottom: 18px;\r\n\
	padding: 0;\r\n\
}\r\n\
\r\n\
.widget_twentyfourteen_ephemera .hentry {\r\n\
	margin: 0;\r\n\
	max-width: 100%;\r\n\
}\r\n\
\r\n\
.widget_twentyfourteen_ephemera .entry-title,\r\n\
.widget_twentyfourteen_ephemera .entry-meta,\r\n\
.widget_twentyfourteen_ephemera .wp-caption-text,\r\n\
.widget_twentyfourteen_ephemera .post-format-archive-link,\r\n\
.widget_twentyfourteen_ephemera .entry-content table {\r\n\
	font-size: 12px;\r\n\
	line-height: 1.5;\r\n\
}\r\n\
\r\n\
.widget_twentyfourteen_ephemera .entry-title {\r\n\
	display: inline;\r\n\
	font-weight: 400;\r\n\
}\r\n\
\r\n\
.widget_twentyfourteen_ephemera .entry-meta {\r\n\
	margin-bottom: 18px;\r\n\
}\r\n\
\r\n\
.widget_twentyfourteen_ephemera .entry-meta a {\r\n\
	color: rgba(255, 255, 255, 0.7);\r\n\
}\r\n\
\r\n\
.widget_twentyfourteen_ephemera .entry-meta a:hover {\r\n\
	color: #41a62a;\r\n\
}\r\n\
\r\n\
.widget_twentyfourteen_ephemera .entry-content ul,\r\n\
.widget_twentyfourteen_ephemera .entry-content ol {\r\n\
	margin: 0 0 18px 20px;\r\n\
}\r\n\
\r\n\
.widget_twentyfourteen_ephemera .entry-content ul {\r\n\
	list-style: disc;\r\n\
}\r\n\
\r\n\
.widget_twentyfourteen_ephemera .entry-content ol {\r\n\
	list-style: decimal;\r\n\
}\r\n\
\r\n\
.widget_twentyfourteen_ephemera .entry-content li > ul,\r\n\
.widget_twentyfourteen_ephemera .entry-content li > ol {\r\n\
	margin: 0 0 0 20px;\r\n\
}\r\n\
\r\n\
.widget_twentyfourteen_ephemera .entry-content th,\r\n\
.widget_twentyfourteen_ephemera .entry-content td {\r\n\
	padding: 6px;\r\n\
}\r\n\
\r\n\
.widget_twentyfourteen_ephemera .post-format-archive-link {\r\n\
	font-weight: 700;\r\n\
	text-transform: uppercase;\r\n\
}\r\n\
\r\n\
/* List Style Widgets*/\r\n\
\r\n\
.widget_archive li,\r\n\
.widget_categories li,\r\n\
.widget_links li,\r\n\
.widget_meta li,\r\n\
.widget_nav_menu li,\r\n\
.widget_pages li,\r\n\
.widget_recent_comments li,\r\n\
.widget_recent_entries li {\r\n\
	border-top: 1px solid rgba(255, 255, 255, 0.2);\r\n\
	padding: 8px 0 9px;\r\n\
}\r\n\
\r\n\
.widget_archive li:first-child,\r\n\
.widget_categories li:first-child,\r\n\
.widget_links li:first-child,\r\n\
.widget_meta li:first-child,\r\n\
.widget_nav_menu li:first-child,\r\n\
.widget_pages li:first-child,\r\n\
.widget_recent_comments li:first-child,\r\n\
.widget_recent_entries li:first-child {\r\n\
	border-top: 0;\r\n\
}\r\n\
\r\n\
.widget_categories li ul,\r\n\
.widget_nav_menu li ul,\r\n\
.widget_pages li ul {\r\n\
	border-top: 1px solid rgba(255, 255, 255, 0.2);\r\n\
	margin-top: 9px;\r\n\
}\r\n\
\r\n\
.widget_categories li li:last-child,\r\n\
.widget_nav_menu li li:last-child,\r\n\
.widget_pages li li:last-child {\r\n\
	padding-bottom: 0;\r\n\
}\r\n\
\r\n\
/* Recent Posts Widget */\r\n\
\r\n\
.widget_recent_entries .post-date {\r\n\
	display: block;\r\n\
}\r\n\
\r\n\
/* RSS Widget */\r\n\
\r\n\
.rsswidget img {\r\n\
	margin-top: -4px;\r\n\
}\r\n\
\r\n\
.rssSummary {\r\n\
	margin: 9px 0;\r\n\
}\r\n\
\r\n\
.rss-date {\r\n\
	display: block;\r\n\
}\r\n\
\r\n\
.widget_rss li {\r\n\
	margin-bottom: 18px;\r\n\
}\r\n\
\r\n\
.widget_rss li:last-child {\r\n\
	margin-bottom: 0;\r\n\
}\r\n\
\r\n\
/* Text Widget */\r\n\
\r\n\
.widget_text > div > :last-child {\r\n\
	margin-bottom: 0;\r\n\
}\r\n\
\r\n\
\r\n\
/**\r\n\
 * 7.2 Content Sidebar Widgets\r\n\
 * -----------------------------------------------------------------------------\r\n\
 */\r\n\
\r\n\
.content-sidebar .widget a {\r\n\
	color: #24890d;\r\n\
}\r\n\
\r\n\
.content-sidebar .widget a:hover {\r\n\
	color: #41a62a;\r\n\
}\r\n\
\r\n\
.content-sidebar .widget pre {\r\n\
	border-color: rgba(0, 0, 0, 0.1);\r\n\
}\r\n\
\r\n\
.content-sidebar .widget mark,\r\n\
.content-sidebar .widget ins {\r\n\
	color: #2b2b2b;\r\n\
}\r\n\
\r\n\
.content-sidebar .widget abbr[title] {\r\n\
	border-color: #2b2b2b;\r\n\
}\r\n\
\r\n\
.content-sidebar .widget fieldset {\r\n\
	border-color: rgba(0, 0, 0, 0.1);\r\n\
}\r\n\
\r\n\
.content-sidebar .widget blockquote {\r\n\
	color: #767676;\r\n\
}\r\n\
\r\n\
.content-sidebar .widget blockquote cite {\r\n\
	color: #2b2b2b;\r\n\
}\r\n\
\r\n\
.content-sidebar .widget li > ol,\r\n\
.content-sidebar .widget li > ul {\r\n\
	margin-left: 18px;\r\n\
}\r\n\
\r\n\
.content-sidebar .widget table,\r\n\
.content-sidebar .widget th,\r\n\
.content-sidebar .widget td {\r\n\
	border-color: rgba(0, 0, 0, 0.1);\r\n\
}\r\n\
\r\n\
.content-sidebar .widget del {\r\n\
	color: #767676;\r\n\
}\r\n\
\r\n\
.content-sidebar .widget hr {\r\n\
	background-color: rgba(0, 0, 0, 0.1);\r\n\
}\r\n\
\r\n\
.content-sidebar .widget input,\r\n\
.content-sidebar .widget textarea {\r\n\
	background-color: #fff;\r\n\
	border-color: rgba(0, 0, 0, 0.1);\r\n\
	color: #2b2b2b;\r\n\
}\r\n\
\r\n\
.content-sidebar .widget input:focus,\r\n\
.content-sidebar .widget textarea:focus {\r\n\
	border-color: rgba(0, 0, 0, 0.3);\r\n\
}\r\n\
\r\n\
.content-sidebar .widget input[type="button"],\r\n\
.content-sidebar .widget input[type="reset"],\r\n\
.content-sidebar .widget input[type="submit"] {\r\n\
	background-color: #24890d;\r\n\
	border: 0;\r\n\
	color: #fff;\r\n\
}\r\n\
\r\n\
.content-sidebar .widget input[type="button"]:hover,\r\n\
.content-sidebar .widget input[type="button"]:focus,\r\n\
.content-sidebar .widget input[type="reset"]:hover,\r\n\
.content-sidebar .widget input[type="reset"]:focus,\r\n\
.content-sidebar .widget input[type="submit"]:hover,\r\n\
.content-sidebar .widget input[type="submit"]:focus {\r\n\
	background-color: #41a62a;\r\n\
}\r\n\
\r\n\
.content-sidebar .widget input[type="button"]:active,\r\n\
.content-sidebar .widget input[type="reset"]:active,\r\n\
.content-sidebar .widget input[type="submit"]:active {\r\n\
	background-color: #55d737;\r\n\
}\r\n\
\r\n\
.content-sidebar .widget .wp-caption {\r\n\
	color: #767676;\r\n\
}\r\n\
\r\n\
.content-sidebar .widget .widget-title {\r\n\
	border-top: 5px solid #000;\r\n\
	color: #2b2b2b;\r\n\
	font-size: 14px;\r\n\
	font-weight: 900;\r\n\
	margin: 0 0 18px;\r\n\
	padding-top: 7px;\r\n\
	text-transform: uppercase;\r\n\
}\r\n\
\r\n\
.content-sidebar .widget .widget-title a {\r\n\
	color: #2b2b2b;\r\n\
}\r\n\
\r\n\
.content-sidebar .widget .widget-title a:hover {\r\n\
	color: #41a62a;\r\n\
}\r\n\
\r\n\
/* List Style Widgets*/\r\n\
\r\n\
.content-sidebar .widget_archive li,\r\n\
.content-sidebar .widget_categories li,\r\n\
.content-sidebar .widget_links li,\r\n\
.content-sidebar .widget_meta li,\r\n\
.content-sidebar .widget_nav_menu li,\r\n\
.content-sidebar .widget_pages li,\r\n\
.content-sidebar .widget_recent_comments li,\r\n\
.content-sidebar .widget_recent_entries li,\r\n\
.content-sidebar .widget_categories li ul,\r\n\
.content-sidebar .widget_nav_menu li ul,\r\n\
.content-sidebar .widget_pages li ul {\r\n\
	border-color: rgba(0, 0, 0, 0.1);\r\n\
}\r\n\
\r\n\
/* Calendar Widget */\r\n\
\r\n\
.content-sidebar .widget_calendar caption {\r\n\
	color: #2b2b2b;\r\n\
	font-weight: 900;\r\n\
}\r\n\
\r\n\
.content-sidebar .widget_calendar thead th {\r\n\
	background-color: rgba(0, 0, 0, 0.02);\r\n\
}\r\n\
\r\n\
.content-sidebar .widget_calendar tbody a,\r\n\
.content-sidebar .widget_calendar tbody a:hover {\r\n\
	color: #fff;\r\n\
}\r\n\
\r\n\
/* Ephemera widget*/\r\n\
\r\n\
.content-sidebar .widget_twentyfourteen_ephemera .widget-title {\r\n\
	line-height: 1.2857142857;\r\n\
	padding-top: 1px;\r\n\
}\r\n\
\r\n\
.content-sidebar .widget_twentyfourteen_ephemera .widget-title:before {\r\n\
	background-color: #000;\r\n\
	color: #fff;\r\n\
	margin: -1px 9px 0 0;\r\n\
	padding: 6px 0 9px;\r\n\
	text-align: center;\r\n\
	vertical-align: middle;\r\n\
	width: 36px;\r\n\
}\r\n\
\r\n\
.content-sidebar .widget_twentyfourteen_ephemera .video.widget-title:before {\r\n\
	content: "\f104";\r\n\
}\r\n\
\r\n\
.content-sidebar .widget_twentyfourteen_ephemera .audio.widget-title:before {\r\n\
	content: "\f109";\r\n\
}\r\n\
\r\n\
.content-sidebar .widget_twentyfourteen_ephemera .image.widget-title:before {\r\n\
	content: "\f473";\r\n\
}\r\n\
\r\n\
.content-sidebar .widget_twentyfourteen_ephemera .gallery.widget-title:before {\r\n\
	content: "\f103";\r\n\
}\r\n\
\r\n\
.content-sidebar .widget_twentyfourteen_ephemera .aside.widget-title:before {\r\n\
	content: "\f101";\r\n\
}\r\n\
\r\n\
.content-sidebar .widget_twentyfourteen_ephemera .quote.widget-title:before {\r\n\
	content: "\f106";\r\n\
}\r\n\
\r\n\
.content-sidebar .widget_twentyfourteen_ephemera .link.widget-title:before {\r\n\
	content: "\f107";\r\n\
}\r\n\
\r\n\
.content-sidebar .widget_twentyfourteen_ephemera > ol > li {\r\n\
	border-bottom: 1px solid rgba(0, 0, 0, 0.1);\r\n\
}\r\n\
\r\n\
.content-sidebar .widget_twentyfourteen_ephemera .entry-meta {\r\n\
	color: #ccc;\r\n\
}\r\n\
\r\n\
.content-sidebar .widget_twentyfourteen_ephemera .entry-meta a {\r\n\
	color: #767676;\r\n\
}\r\n\
\r\n\
.content-sidebar .widget_twentyfourteen_ephemera .entry-meta a:hover {\r\n\
	color: #41a62a;\r\n\
}\r\n\
\r\n\
.content-sidebar.widget_twentyfourteen_ephemera blockquote cite {\r\n\
	font-size: 13px;\r\n\
	line-height: 1.3846153846;\r\n\
}\r\n\
\r\n\
.content-sidebar .widget_twentyfourteen_ephemera .post-format-archive-link {\r\n\
	font-weight: 900;\r\n\
}\r\n\
\r\n\
\r\n\
/**\r\n\
 * 8.0 Footer\r\n\
 * -----------------------------------------------------------------------------\r\n\
 */\r\n\
\r\n\
#supplementary {\r\n\
	padding: 0 10px;\r\n\
}\r\n\
\r\n\
.site-footer,\r\n\
.site-info,\r\n\
.site-info a {\r\n\
	color: rgba(255, 255, 255, 0.7);\r\n\
}\r\n\
\r\n\
.site-footer {\r\n\
	background-color: #000;\r\n\
	font-size: 12px;\r\n\
	position: relative;\r\n\
	z-index: 3;\r\n\
}\r\n\
\r\n\
.footer-sidebar {\r\n\
	padding-top: 48px;\r\n\
}\r\n\
\r\n\
.site-info {\r\n\
	padding: 15px 10px;\r\n\
}\r\n\
\r\n\
#supplementary + .site-info {\r\n\
	border-top: 1px solid rgba(255, 255, 255, 0.2);\r\n\
}\r\n\
\r\n\
.site-info a:hover {\r\n\
	color: #41a62a;\r\n\
}\r\n\
\r\n\
\r\n\
/**\r\n\
 * 9.0 Featured Content\r\n\
 * -----------------------------------------------------------------------------\r\n\
 */\r\n\
\r\n\
.featured-content {\r\n\
	background: #000 url(images/pattern-dark.svg) repeat fixed;\r\n\
	-webkit-box-sizing: border-box;\r\n\
	-moz-box-sizing:    border-box;\r\n\
	box-sizing:         border-box;\r\n\
	position: relative;\r\n\
	width: 100%;\r\n\
}\r\n\
\r\n\
.featured-content-inner {\r\n\
	overflow: hidden;\r\n\
}\r\n\
\r\n\
.featured-content .hentry {\r\n\
	color: #fff;\r\n\
	margin: 0;\r\n\
	max-width: 100%;\r\n\
	width: 100%;\r\n\
}\r\n\
\r\n\
.featured-content .post-thumbnail,\r\n\
.featured-content .post-thumbnail:hover {\r\n\
	background: transparent;\r\n\
}\r\n\
\r\n\
.featured-content .post-thumbnail {\r\n\
	display: block;\r\n\
	position: relative;\r\n\
	padding-top: 55.357142857%;\r\n\
	overflow: hidden;\r\n\
}\r\n\
\r\n\
.featured-content .post-thumbnail img {\r\n\
	left: 0;\r\n\
	position: absolute;\r\n\
	top: 0;\r\n\
}\r\n\
\r\n\
.featured-content .entry-header {\r\n\
	background-color: #000;\r\n\
	-webkit-box-sizing: border-box;\r\n\
	-moz-box-sizing:    border-box;\r\n\
	box-sizing:         border-box;\r\n\
	min-height: 96px;\r\n\
	overflow: hidden;\r\n\
	padding: 24px 10px;\r\n\
}\r\n\
\r\n\
.featured-content a {\r\n\
	color: #fff;\r\n\
}\r\n\
\r\n\
.featured-content a:hover {\r\n\
	color: #41a62a;\r\n\
}\r\n\
\r\n\
.featured-content .entry-meta {\r\n\
	color: #fff;\r\n\
	font-size: 11px;\r\n\
	font-weight: 700;\r\n\
	line-height: 1.0909090909;\r\n\
	margin-bottom: 12px;\r\n\
}\r\n\
\r\n\
.featured-content .cat-links {\r\n\
	font-weight: 700;\r\n\
}\r\n\
\r\n\
.featured-content .entry-title {\r\n\
	font-size: 18px;\r\n\
	font-weight: 300;\r\n\
	line-height: 1.3333333333;\r\n\
	margin: 0;\r\n\
	text-transform: uppercase;\r\n\
}\r\n\
\r\n\
\r\n\
/* Slider */\r\n\
\r\n\
.slider .featured-content .hentry {\r\n\
	-webkit-backface-visibility: hidden;\r\n\
	display: none;\r\n\
	position: relative;\r\n\
}\r\n\
\r\n\
.slider .featured-content .post-thumbnail {\r\n\
	padding-top: 55.49132947%;\r\n\
}\r\n\
\r\n\
.slider-control-paging {\r\n\
	background-color: #000;\r\n\
	-webkit-box-sizing: border-box;\r\n\
	-moz-box-sizing:    border-box;\r\n\
	box-sizing:         border-box;\r\n\
	float: left;\r\n\
	list-style: none;\r\n\
	margin: -24px 0 0 0;\r\n\
	position: relative;\r\n\
	width: 100%;\r\n\
	z-index: 3;\r\n\
}\r\n\
\r\n\
.slider-control-paging li {\r\n\
	float: left;\r\n\
	margin: 2px 4px 2px 0;\r\n\
}\r\n\
\r\n\
.slider-control-paging li:last-child {\r\n\
	margin-right: 0;\r\n\
}\r\n\
\r\n\
.slider-control-paging a {\r\n\
	cursor: pointer;\r\n\
	display: block;\r\n\
	height: 44px;\r\n\
	position: relative;\r\n\
	text-indent: -999em;\r\n\
	width: 44px;\r\n\
}\r\n\
\r\n\
.slider-control-paging a:before {\r\n\
	background-color: #4d4d4d;\r\n\
	content: "";\r\n\
	height: 12px;\r\n\
	left: 10px;\r\n\
	position: absolute;\r\n\
	top: 16px;\r\n\
	width: 12px;\r\n\
}\r\n\
\r\n\
.slider-control-paging a:hover:before {\r\n\
	background-color: #41a62a;\r\n\
}\r\n\
\r\n\
.slider-control-paging .slider-active:before,\r\n\
.slider-control-paging .slider-active:hover:before {\r\n\
	background-color: #24890d;\r\n\
}\r\n\
\r\n\
.slider-direction-nav {\r\n\
	clear: both;\r\n\
	list-style: none;\r\n\
	margin: 0;\r\n\
	position: relative;\r\n\
	width: 100%;\r\n\
	z-index: 3;\r\n\
}\r\n\
\r\n\
.slider-direction-nav li {\r\n\
	border-color: #fff;\r\n\
	border-style: solid;\r\n\
	border-width: 2px 1px 0 0;\r\n\
	-webkit-box-sizing: border-box;\r\n\
	-moz-box-sizing:    border-box;\r\n\
	box-sizing:         border-box;\r\n\
	float: left;\r\n\
	text-align: center;\r\n\
	width: 50%;\r\n\
}\r\n\
\r\n\
.slider-direction-nav li:last-child {\r\n\
	border-width: 2px 0 0 1px;\r\n\
}\r\n\
\r\n\
.slider-direction-nav a {\r\n\
	background-color: #000;\r\n\
	display: block;\r\n\
	font-size: 0;\r\n\
	height: 46px;\r\n\
}\r\n\
\r\n\
.slider-direction-nav a:hover {\r\n\
	background-color: #24890d;\r\n\
}\r\n\
\r\n\
.slider-direction-nav a:before {\r\n\
	color: #fff;\r\n\
	content: "\f430";\r\n\
	font-size: 32px;\r\n\
	line-height: 46px;\r\n\
}\r\n\
\r\n\
.slider-direction-nav .slider-next:before {\r\n\
	content: "\f429";\r\n\
}\r\n\
\r\n\
.slider-direction-nav .slider-disabled {\r\n\
	display: none;\r\n\
}\r\n\
\r\n\
\r\n\
/**\r\n\
 * 10.0 Multisite\r\n\
 * -----------------------------------------------------------------------------\r\n\
 */\r\n\
\r\n\
.site-main .widecolumn {\r\n\
	padding-top: 72px;\r\n\
	width: auto;\r\n\
}\r\n\
.site-main .mu_register,\r\n\
.widecolumn > h2,\r\n\
.widecolumn > form {\r\n\
	margin: 0 auto 48px;\r\n\
	max-width: 474px;\r\n\
	padding: 0 30px;\r\n\
}\r\n\
\r\n\
.site-main .mu_register #blog_title,\r\n\
.site-main .mu_register #user_email,\r\n\
.site-main .mu_register #blogname,\r\n\
.site-main .mu_register #user_name {\r\n\
	font-size: inherit;\r\n\
	width: 90%;\r\n\
}\r\n\
\r\n\
.site-main .mu_register input[type="submit"],\r\n\
.widecolumn #submit {\r\n\
	font-size: inherit;\r\n\
	width: auto;\r\n\
}\r\n\
\r\n\
\r\n\
/**\r\n\
 * 11.0 Media Queries\r\n\
 * -----------------------------------------------------------------------------\r\n\
 */\r\n\
\r\n\
/* Does the same thing as <meta name="viewport" content="width=device-width">,\r\n\
 * but in the future W3C standard way. -ms- prefix is required for IE10+ to\r\n\
 * render responsive styling in Windows 8 "snapped" views; IE10+ does not honor\r\n\
 * the meta tag. See https://core.trac.wordpress.org/ticket/25888.\r\n\
 */\r\n\
@-ms-viewport {\r\n\
	width: device-width;\r\n\
}\r\n\
\r\n\
@viewport {\r\n\
	width: device-width;\r\n\
}\r\n\
\r\n\
@media screen and (max-width: 400px) {\r\n\
	.list-view .site-content .post-thumbnail {\r\n\
		background: none;\r\n\
		width: auto;\r\n\
		z-index: 2;\r\n\
	}\r\n\
\r\n\
	.list-view .site-content .post-thumbnail img {\r\n\
		float: left;\r\n\
		margin: 0 10px 3px 0;\r\n\
		width: 84px;\r\n\
	}\r\n\
\r\n\
	.list-view .site-content .entry-header {\r\n\
		background-color: transparent;\r\n\
		padding: 0;\r\n\
	}\r\n\
\r\n\
	.list-view .content-area {\r\n\
		padding: 0 10px;\r\n\
	}\r\n\
\r\n\
	.list-view .site-content .hentry {\r\n\
		border-bottom: 1px solid rgba(0, 0, 0, 0.1);\r\n\
		margin: 0;\r\n\
		min-height: 60px;\r\n\
		padding: 12px 0 9px;\r\n\
	}\r\n\
\r\n\
	.list-view .site-content .cat-links,\r\n\
	.list-view .site-content .type-post .entry-content,\r\n\
	.list-view .site-content .type-page .entry-content,\r\n\
	.list-view .site-content .type-post .entry-summary,\r\n\
	.list-view .site-content .type-page .entry-summary,\r\n\
	.list-view .site-content footer.entry-meta {\r\n\
		display: none;\r\n\
	}\r\n\
\r\n\
	.list-view .site-content .entry-title {\r\n\
		clear: none;\r\n\
		font-size: 15px;\r\n\
		font-weight: 900;\r\n\
		line-height: 1.2;\r\n\
		margin-bottom: 6px;\r\n\
		text-transform: none;\r\n\
	}\r\n\
\r\n\
	.list-view .site-content .format-aside .entry-title,\r\n\
	.list-view .site-content .format-link .entry-title,\r\n\
	.list-view .site-content .format-quote .entry-title {\r\n\
		display: block;\r\n\
	}\r\n\
\r\n\
	.list-view .site-content .entry-meta {\r\n\
		background-color: transparent;\r\n\
		clear: none;\r\n\
		margin: 0;\r\n\
		text-transform: none;\r\n\
	}\r\n\
\r\n\
	.archive-header,\r\n\
	.page-header {\r\n\
		border-bottom: 1px solid rgba(0, 0, 0, 0.1);\r\n\
		margin: 24px auto 0;\r\n\
		padding-bottom: 24px;\r\n\
	}\r\n\
\r\n\
	.error404 .page-header {\r\n\
		border-bottom: 0;\r\n\
		margin: 0 auto 24px;\r\n\
		padding: 0 10px;\r\n\
	}\r\n\
}\r\n\
\r\n\
@media screen and (min-width: 401px) {\r\n\
	a.post-thumbnail:hover img {\r\n\
		opacity: 0.85;\r\n\
	}\r\n\
\r\n\
	.full-size-link:before,\r\n\
	.parent-post-link:before,\r\n\
	.site-content span + .byline:before,\r\n\
	.site-content span + .comments-link:before,\r\n\
	.site-content span + .edit-link:before,\r\n\
	.site-content span + .entry-date:before {\r\n\
		content: "";\r\n\
	}\r\n\
\r\n\
	.attachment span.entry-date:before,\r\n\
	.entry-content .edit-link a:before,\r\n\
	.entry-meta .edit-link a:before,\r\n\
	.site-content .byline a:before,\r\n\
	.site-content .comments-link a:before,\r\n\
	.site-content .entry-date a:before,\r\n\
	.site-content .featured-post:before,\r\n\
	.site-content .full-size-link a:before,\r\n\
	.site-content .parent-post-link a:before,\r\n\
	.site-content .post-format a:before {\r\n\
		-webkit-font-smoothing: antialiased;\r\n\
		display: inline-block;\r\n\
		font: normal 16px/1 Genericons;\r\n\
		text-decoration: inherit;\r\n\
		vertical-align: text-bottom;\r\n\
	}\r\n\
\r\n\
	.site-content .entry-meta > span {\r\n\
		margin-right: 10px;\r\n\
	}\r\n\
\r\n\
	.site-content .format-video .post-format a:before {\r\n\
		content: "\f104";\r\n\
	}\r\n\
\r\n\
	.site-content .format-audio .post-format a:before {\r\n\
		content: "\f109";\r\n\
	}\r\n\
\r\n\
	.site-content .format-image .post-format a:before {\r\n\
		content: "\f473";\r\n\
	}\r\n\
\r\n\
	.site-content .format-quote .post-format a:before {\r\n\
		content: "\f106";\r\n\
		margin-right: 2px;\r\n\
	}\r\n\
\r\n\
	.site-content .format-gallery .post-format a:before {\r\n\
		content: "\f103";\r\n\
		margin-right: 4px;\r\n\
	}\r\n\
\r\n\
	.site-content .format-aside .post-format a:before {\r\n\
		content: "\f101";\r\n\
		margin-right: 2px;\r\n\
	}\r\n\
\r\n\
	.site-content .format-link .post-format a:before {\r\n\
		content: "\f107";\r\n\
		position: relative;\r\n\
		top: 1px;\r\n\
	}\r\n\
\r\n\
	.site-content .featured-post:before {\r\n\
		content: "\f308";\r\n\
		margin-right: 3px;\r\n\
		position: relative;\r\n\
		top: 1px;\r\n\
	}\r\n\
\r\n\
	.site-content .entry-date a:before,\r\n\
	.attachment .site-content span.entry-date:before {\r\n\
		content: "\f303";\r\n\
		margin-right: 1px;\r\n\
		position: relative;\r\n\
		top: 1px;\r\n\
	}\r\n\
\r\n\
	.site-content .byline a:before {\r\n\
		content: "\f304";\r\n\
	}\r\n\
\r\n\
	.site-content .comments-link a:before {\r\n\
		content: "\f300";\r\n\
		margin-right: 2px;\r\n\
	}\r\n\
\r\n\
	.entry-content .edit-link a:before,\r\n\
	.entry-meta .edit-link a:before {\r\n\
		content: "\f411";\r\n\
	}\r\n\
\r\n\
	.site-content .full-size-link a:before {\r\n\
		content: "\f402";\r\n\
		margin-right: 1px;\r\n\
	}\r\n\
\r\n\
	.site-content .parent-post-link a:before {\r\n\
		content: "\f301";\r\n\
	}\r\n\
\r\n\
	.list-view .site-content .hentry {\r\n\
		border-top: 1px solid rgba(0, 0, 0, 0.1);\r\n\
		padding-top: 48px;\r\n\
	}\r\n\
\r\n\
	.list-view .site-content .hentry:first-of-type,\r\n\
	.list-view .site-content .hentry.has-post-thumbnail {\r\n\
		border-top: 0;\r\n\
		padding-top: 0;\r\n\
	}\r\n\
\r\n\
	.archive-header,\r\n\
	.page-header {\r\n\
		margin: 0 auto 60px;\r\n\
		padding: 0 10px;\r\n\
	}\r\n\
\r\n\
	.error404 .page-header {\r\n\
		margin-bottom: 24px;\r\n\
	}\r\n\
}\r\n\
\r\n\
@media screen and (min-width: 594px) {\r\n\
	.site-content .entry-header {\r\n\
		padding-right: 30px;\r\n\
		padding-left: 30px;\r\n\
	}\r\n\
\r\n\
	.site-content .has-post-thumbnail .entry-header {\r\n\
		margin-top: -48px;\r\n\
	}\r\n\
}\r\n\
\r\n\
@media screen and (min-width: 673px) {\r\n\
	.header-main {\r\n\
		padding: 0 30px;\r\n\
	}\r\n\
\r\n\
	.search-toggle {\r\n\
		margin-right: 18px;\r\n\
	}\r\n\
\r\n\
	.search-box .search-field {\r\n\
		width: 50%;\r\n\
	}\r\n\
\r\n\
	.content-area {\r\n\
		float: left;\r\n\
		width: 100%;\r\n\
	}\r\n\
\r\n\
	.site-content {\r\n\
		margin-right: 33.33333333%;\r\n\
	}\r\n\
\r\n\
	.site-content .has-post-thumbnail .entry-header {\r\n\
		margin-top: 0;\r\n\
	}\r\n\
\r\n\
	.archive-header,\r\n\
	.comments-area,\r\n\
	.image-navigation,\r\n\
	.page-header,\r\n\
	.page-content,\r\n\
	.post-navigation,\r\n\
	.site-content .entry-content,\r\n\
	.site-content .entry-summary,\r\n\
	.site-content footer.entry-meta {\r\n\
		padding-right: 30px;\r\n\
		padding-left: 30px;\r\n\
	}\r\n\
\r\n\
	.singular .site-content .hentry.has-post-thumbnail {\r\n\
		margin-top: 0;\r\n\
	}\r\n\
\r\n\
	.full-width .site-content {\r\n\
		margin-right: 0;\r\n\
	}\r\n\
\r\n\
	.full-width .site-content .has-post-thumbnail .entry-header,\r\n\
	.full-width .site-content .hentry.has-post-thumbnail:first-child {\r\n\
		margin-top: -48px;\r\n\
	}\r\n\
\r\n\
	#secondary,\r\n\
	#supplementary {\r\n\
		padding: 0 30px;\r\n\
	}\r\n\
\r\n\
	.content-sidebar {\r\n\
		border: 0;\r\n\
		float: right;\r\n\
		margin-left: -33.33333333%;\r\n\
		padding: 48px 30px 24px;\r\n\
		position: relative;\r\n\
		width: 33.33333333%;\r\n\
	}\r\n\
\r\n\
	.grid .featured-content .hentry {\r\n\
		float: left;\r\n\
		width: 50%;\r\n\
	}\r\n\
\r\n\
	.grid .featured-content .hentry:nth-child( 2n+1 ) {\r\n\
		clear: both;\r\n\
	}\r\n\
\r\n\
	.grid .featured-content .entry-header {\r\n\
		border-color: #000;\r\n\
		border-style: solid;\r\n\
		border-width: 12px 10px;\r\n\
		height: 96px;\r\n\
		padding: 0;\r\n\
	}\r\n\
\r\n\
	.slider .featured-content .entry-title {\r\n\
		font-size: 22px;\r\n\
		line-height: 1.0909090909;\r\n\
	}\r\n\
\r\n\
	.slider .featured-content .entry-header {\r\n\
		min-height: inherit;\r\n\
		padding: 24px 30px 48px;\r\n\
		position: absolute;\r\n\
		left: 0;\r\n\
		bottom: 0;\r\n\
		width: 50%;\r\n\
		z-index: 3;\r\n\
	}\r\n\
\r\n\
	.slider-control-paging {\r\n\
		background: transparent;\r\n\
		margin-top: -48px;\r\n\
		padding-left: 20px;\r\n\
		width: 50%;\r\n\
	}\r\n\
\r\n\
	.slider-direction-nav {\r\n\
		clear: none;\r\n\
		float: right;\r\n\
		margin-top: -48px;\r\n\
		width: 98px;\r\n\
	}\r\n\
\r\n\
	.slider-direction-nav li {\r\n\
		border: 0;\r\n\
		padding: 0 1px 0 0;\r\n\
	}\r\n\
\r\n\
	.slider-direction-nav li:last-child {\r\n\
		padding: 0 0 0 1px;\r\n\
	}\r\n\
\r\n\
	.slider-direction-nav a {\r\n\
		height: 48px;\r\n\
	}\r\n\
\r\n\
	.slider-direction-nav a:before {\r\n\
		line-height: 48px;\r\n\
	}\r\n\
\r\n\
	.site-info {\r\n\
		padding: 15px 30px;\r\n\
	}\r\n\
}\r\n\
\r\n\
@media screen and (min-width: 783px) {\r\n\
	.site-title {\r\n\
		/* Search-toggle width = 48px */\r\n\
		max-width: -webkit-calc(100% - 48px);\r\n\
		max-width:         calc(100% - 48px);\r\n\
	}\r\n\
\r\n\
	.header-main {\r\n\
		padding-right: 0;\r\n\
	}\r\n\
\r\n\
	.search-toggle {\r\n\
		margin-right: 0;\r\n\
	}\r\n\
\r\n\
	/* Fixed Header */\r\n\
\r\n\
	.masthead-fixed .site-header {\r\n\
		position: fixed;\r\n\
		top: 0;\r\n\
	}\r\n\
\r\n\
	.admin-bar.masthead-fixed .site-header {\r\n\
		top: 32px;\r\n\
	}\r\n\
\r\n\
	.masthead-fixed .site-main {\r\n\
		margin-top: 48px;\r\n\
	}\r\n\
\r\n\
	/* Navigation */\r\n\
\r\n\
	.site-navigation li .current_page_item > a,\r\n\
	.site-navigation li .current_page_ancestor > a,\r\n\
	.site-navigation li .current-menu-item > a,\r\n\
	.site-navigation li .current-menu-ancestor > a {\r\n\
		color: #fff;\r\n\
	}\r\n\
\r\n\
	/* Primary Navigation */\r\n\
\r\n\
	.primary-navigation {\r\n\
		float: right;\r\n\
		font-size: 11px;\r\n\
		margin: 0 1px 0 -12px;\r\n\
		padding: 0;\r\n\
		text-transform: uppercase;\r\n\
	}\r\n\
\r\n\
	.primary-navigation .menu-toggle {\r\n\
		display: none;\r\n\
		padding: 0;\r\n\
	}\r\n\
\r\n\
	.primary-navigation .nav-menu {\r\n\
		border-bottom: 0;\r\n\
		display: block;\r\n\
	}\r\n\
\r\n\
	.primary-navigation.toggled-on {\r\n\
		border-bottom: 0;\r\n\
		margin: 0;\r\n\
		padding: 0;\r\n\
	}\r\n\
\r\n\
	.primary-navigation li {\r\n\
		border: 0;\r\n\
		display: inline-block;\r\n\
		height: 48px;\r\n\
		line-height: 48px;\r\n\
		position: relative;\r\n\
	}\r\n\
\r\n\
	.primary-navigation a {\r\n\
		display: inline-block;\r\n\
		padding: 0 12px;\r\n\
		white-space: nowrap;\r\n\
	}\r\n\
\r\n\
	.primary-navigation ul ul {\r\n\
		background-color: #24890d;\r\n\
		float: left;\r\n\
		margin: 0;\r\n\
		position: absolute;\r\n\
		top: 48px;\r\n\
		left: -999em;\r\n\
		z-index: 99999;\r\n\
	}\r\n\
\r\n\
	.primary-navigation li li {\r\n\
		border: 0;\r\n\
		display: block;\r\n\
		height: auto;\r\n\
		line-height: 1.0909090909;\r\n\
	}\r\n\
\r\n\
	.primary-navigation ul ul ul {\r\n\
		left: -999em;\r\n\
		top: 0;\r\n\
	}\r\n\
\r\n\
	.primary-navigation ul ul a {\r\n\
		padding: 18px 12px;\r\n\
		white-space: normal;\r\n\
		width: 176px;\r\n\
	}\r\n\
\r\n\
	.primary-navigation li:hover > a,\r\n\
	.primary-navigation li.focus > a {\r\n\
		background-color: #24890d;\r\n\
		color: #fff;\r\n\
	}\r\n\
\r\n\
	.primary-navigation ul ul a:hover,\r\n\
	.primary-navigation ul ul li.focus > a {\r\n\
		background-color: #41a62a;\r\n\
	}\r\n\
\r\n\
	.primary-navigation ul li:hover > ul,\r\n\
	.primary-navigation ul li.focus > ul {\r\n\
		left: auto;\r\n\
	}\r\n\
\r\n\
	.primary-navigation ul ul li:hover > ul,\r\n\
	.primary-navigation ul ul li.focus > ul {\r\n\
		left: 100%;\r\n\
	}\r\n\
\r\n\
	.primary-navigation .menu-item-has-children > a,\r\n\
	.primary-navigation .page_item_has_children > a {\r\n\
		padding-right: 26px;\r\n\
	}\r\n\
\r\n\
	.primary-navigation .menu-item-has-children > a:after,\r\n\
	.primary-navigation .page_item_has_children > a:after {\r\n\
		-webkit-font-smoothing: antialiased;\r\n\
		content: "\f502";\r\n\
		display: inline-block;\r\n\
		font: normal 8px/1 Genericons;\r\n\
		position: absolute;\r\n\
		right: 12px;\r\n\
		top: 22px;\r\n\
		vertical-align: text-bottom;\r\n\
	}\r\n\
\r\n\
	.primary-navigation li .menu-item-has-children > a,\r\n\
	.primary-navigation li .page_item_has_children > a {\r\n\
		padding-right: 20px;\r\n\
		width: 168px;\r\n\
	}\r\n\
\r\n\
	.primary-navigation .menu-item-has-children li.menu-item-has-children > a:after,\r\n\
	.primary-navigation .menu-item-has-children li.page_item_has_children > a:after,\r\n\
	.primary-navigation .page_item_has_children li.menu-item-has-children > a:after,\r\n\
	.primary-navigation .page_item_has_children li.page_item_has_children > a:after {\r\n\
		content: "\f501";\r\n\
		right: 8px;\r\n\
		top: 20px;\r\n\
	}\r\n\
}\r\n\
\r\n\
@media screen and (min-width: 810px) {\r\n\
	.attachment .entry-attachment .attachment {\r\n\
		margin-right: -168px;\r\n\
		margin-left: -168px;\r\n\
		max-width: 810px;\r\n\
	}\r\n\
\r\n\
	.attachment .site-content .attachment img {\r\n\
		display: block;\r\n\
		margin: 0 auto;\r\n\
	}\r\n\
\r\n\
	.contributor-avatar {\r\n\
		margin-left: -168px;\r\n\
	}\r\n\
\r\n\
	.contributor-summary {\r\n\
		float: left;\r\n\
	}\r\n\
\r\n\
	.full-width .site-content blockquote.alignleft,\r\n\
	.full-width .site-content blockquote.alignright {\r\n\
		width: -webkit-calc(50% + 130px);\r\n\
		width:         calc(50% + 130px);\r\n\
	}\r\n\
\r\n\
	.full-width .site-content blockquote.alignleft,\r\n\
	.full-width .site-content img.size-full.alignleft,\r\n\
	.full-width .site-content img.size-large.alignleft,\r\n\
	.full-width .site-content img.size-medium.alignleft,\r\n\
	.full-width .site-content .wp-caption.alignleft {\r\n\
		margin-left: -168px;\r\n\
	}\r\n\
\r\n\
	.full-width .site-content .alignleft {\r\n\
		clear: left;\r\n\
	}\r\n\
\r\n\
	.full-width .site-content blockquote.alignright,\r\n\
	.full-width .site-content img.size-full.alignright,\r\n\
	.full-width .site-content img.size-large.alignright,\r\n\
	.full-width .site-content img.size-medium.alignright,\r\n\
	.full-width .site-content .wp-caption.alignright {\r\n\
		margin-right: -168px;\r\n\
	}\r\n\
\r\n\
	.full-width .site-content .alignright {\r\n\
		clear: right;\r\n\
	}\r\n\
}\r\n\
\r\n\
@media screen and (min-width: 846px) {\r\n\
	.content-area,\r\n\
	.content-sidebar {\r\n\
		padding-top: 72px;\r\n\
	}\r\n\
\r\n\
	.site-content .has-post-thumbnail .entry-header {\r\n\
		margin-top: -48px;\r\n\
	}\r\n\
\r\n\
	.comment-list .trackback,\r\n\
	.comment-list .pingback,\r\n\
	.comment-list article {\r\n\
		margin-bottom: 36px;\r\n\
		padding-top: 36px;\r\n\
	}\r\n\
\r\n\
	.comment-author .avatar {\r\n\
		height: 34px;\r\n\
		top: 2px;\r\n\
		width: 34px;\r\n\
	}\r\n\
\r\n\
	.comment-author,\r\n\
	.comment-awaiting-moderation,\r\n\
	.comment-content,\r\n\
	.comment-list .reply,\r\n\
	.comment-metadata {\r\n\
		padding-left: 50px;\r\n\
	}\r\n\
\r\n\
	.comment-list .children {\r\n\
		margin-left: 20px;\r\n\
	}\r\n\
\r\n\
	.full-width .site-content .hentry.has-post-thumbnail:first-child {\r\n\
		margin-top: -72px;\r\n\
	}\r\n\
\r\n\
	.featured-content {\r\n\
		margin-bottom: 0;\r\n\
	}\r\n\
}\r\n\
\r\n\
@media screen and (min-width: 1008px) {\r\n\
	.search-box-wrapper {\r\n\
		padding-left: 182px;\r\n\
	}\r\n\
\r\n\
	.main-content {\r\n\
		float: left;\r\n\
	}\r\n\
\r\n\
	.site-content {\r\n\
		margin-right: 29.04761904%;\r\n\
		margin-left: 182px;\r\n\
	}\r\n\
\r\n\
	.site-content .entry-header {\r\n\
		margin-top: 0;\r\n\
	}\r\n\
\r\n\
	.site-content .has-post-thumbnail .entry-header {\r\n\
		margin-top: 0;\r\n\
	}\r\n\
\r\n\
	.content-sidebar {\r\n\
		margin-left: -29.04761904%;\r\n\
		width: 29.04761904%;\r\n\
	}\r\n\
\r\n\
	.site:before {\r\n\
		background-color: #000;\r\n\
		content: "";\r\n\
		display: block;\r\n\
		height: 100%;\r\n\
		min-height: 100%;\r\n\
		position: absolute;\r\n\
		top: 0;\r\n\
		left: 0;\r\n\
		width: 182px;\r\n\
		z-index: 2;\r\n\
	}\r\n\
\r\n\
	#secondary {\r\n\
		background-color: transparent;\r\n\
		border: 0;\r\n\
		clear: none;\r\n\
		float: left;\r\n\
		margin: 0 0 0 -100%;\r\n\
		min-height: 100vh;\r\n\
		width: 122px;\r\n\
	}\r\n\
\r\n\
	.primary-sidebar {\r\n\
		padding-top: 0;\r\n\
	}\r\n\
\r\n\
	.site-description {\r\n\
		display: block;\r\n\
		margin: 0 0 18px;\r\n\
	}\r\n\
\r\n\
	.site-description:empty {\r\n\
		margin: 0;\r\n\
	}\r\n\
\r\n\
	.secondary-navigation {\r\n\
		font-size: 11px;\r\n\
		margin: 0 -30px 48px;\r\n\
		width: 182px;\r\n\
	}\r\n\
\r\n\
	.secondary-navigation li {\r\n\
		border-top: 1px solid rgba(255, 255, 255, 0.2);\r\n\
		position: relative;\r\n\
	}\r\n\
\r\n\
	.secondary-navigation a {\r\n\
		padding: 10px 30px;\r\n\
	}\r\n\
\r\n\
	.secondary-navigation ul ul {\r\n\
		background-color: #24890d;\r\n\
		position: absolute;\r\n\
		top: 0;\r\n\
		left: -999em;\r\n\
		width: 182px;\r\n\
		z-index: 99999;\r\n\
	}\r\n\
\r\n\
	.secondary-navigation li li {\r\n\
		border-top: 0;\r\n\
	}\r\n\
\r\n\
	.secondary-navigation li:hover > a,\r\n\
	.secondary-navigation li.focus > a {\r\n\
		background-color: #24890d;\r\n\
		color: #fff;\r\n\
	}\r\n\
\r\n\
	.secondary-navigation ul ul a:hover,\r\n\
	.secondary-navigation ul ul li.focus > a {\r\n\
		background-color: #41a62a;\r\n\
	}\r\n\
\r\n\
	.secondary-navigation ul li:hover > ul,\r\n\
	.secondary-navigation ul li.focus > ul {\r\n\
		left: 162px;\r\n\
	}\r\n\
\r\n\
	.secondary-navigation .menu-item-has-children > a {\r\n\
		padding-right: 38px;\r\n\
	}\r\n\
\r\n\
	.secondary-navigation .menu-item-has-children > a:after {\r\n\
		-webkit-font-smoothing: antialiased;\r\n\
		content: "\f501";\r\n\
		display: inline-block;\r\n\
		font: normal 8px/1 Genericons;\r\n\
		position: absolute;\r\n\
		right: 26px;\r\n\
		top: 14px;\r\n\
		vertical-align: text-bottom;\r\n\
	}\r\n\
\r\n\
	.footer-sidebar .widget,\r\n\
	.primary-sidebar .widget {\r\n\
		font-size: 12px;\r\n\
		line-height: 1.5;\r\n\
	}\r\n\
\r\n\
	.footer-sidebar .widget {\r\n\
		-webkit-box-sizing: border-box;\r\n\
		-moz-box-sizing:    border-box;\r\n\
		box-sizing:         border-box;\r\n\
		float: left;\r\n\
		padding: 0 30px;\r\n\
		width: 25%;\r\n\
	}\r\n\
\r\n\
	.footer-sidebar .widget h1,\r\n\
	.primary-sidebar .widget h1 {\r\n\
		font-size: 20px;\r\n\
		line-height: 1.2;\r\n\
	}\r\n\
\r\n\
	.footer-sidebar .widget h2,\r\n\
	.primary-sidebar .widget h2 {\r\n\
		font-size: 18px;\r\n\
		line-height: 1.3333333333;\r\n\
	}\r\n\
\r\n\
	.footer-sidebar .widget h3,\r\n\
	.primary-sidebar .widget h3 {\r\n\
		font-size: 16px;\r\n\
		line-height: 1.5;\r\n\
	}\r\n\
\r\n\
	.footer-sidebar .widget h4,\r\n\
	.primary-sidebar .widget h4 {\r\n\
		font-size: 14px;\r\n\
		line-height: 1.7142857142;\r\n\
	}\r\n\
\r\n\
	.footer-sidebar .widget h5,\r\n\
	.primary-sidebar .widget h5 {\r\n\
		font-size: 12px;\r\n\
		line-height: 2;\r\n\
	}\r\n\
\r\n\
	.footer-sidebar .widget h6,\r\n\
	.primary-sidebar .widget h6 {\r\n\
		font-size: 11px;\r\n\
		line-height: 2.1818181818;\r\n\
	}\r\n\
\r\n\
	.footer-sidebar .widget code,\r\n\
	.footer-sidebar .widget kbd,\r\n\
	.footer-sidebar .widget tt,\r\n\
	.footer-sidebar .widget var,\r\n\
	.footer-sidebar .widget samp,\r\n\
	.footer-sidebar .widget pre,\r\n\
	.primary-sidebar .widget code,\r\n\
	.primary-sidebar .widget kbd,\r\n\
	.primary-sidebar .widget tt,\r\n\
	.primary-sidebar .widget var,\r\n\
	.primary-sidebar .widget samp,\r\n\
	.primary-sidebar .widget pre {\r\n\
		font-size: 11px;\r\n\
		line-height: 1.6363636363;\r\n\
	}\r\n\
\r\n\
	.footer-sidebar .widget blockquote,\r\n\
	.primary-sidebar .widget blockquote {\r\n\
		font-size: 14px;\r\n\
		line-height: 1.2857142857;\r\n\
	}\r\n\
\r\n\
	.footer-sidebar .widget blockquote cite,\r\n\
	.primary-sidebar .widget blockquote cite {\r\n\
		font-size: 12px;\r\n\
		line-height: 1.5;\r\n\
	}\r\n\
\r\n\
	.footer-sidebar .widget input,\r\n\
	.footer-sidebar .widget textarea,\r\n\
	.primary-sidebar .widget input,\r\n\
	.primary-sidebar .widget textarea {\r\n\
		font-size: 12px;\r\n\
		padding: 3px 2px 4px 4px;\r\n\
	}\r\n\
\r\n\
	.footer-sidebar .widget input[type="button"],\r\n\
	.footer-sidebar .widget input[type="reset"],\r\n\
	.footer-sidebar .widget input[type="submit"],\r\n\
	.primary-sidebar .widget input[type="button"],\r\n\
	.primary-sidebar .widget input[type="reset"],\r\n\
	.primary-sidebar .widget input[type="submit"] {\r\n\
		padding: 5px 15px 4px;\r\n\
	}\r\n\
\r\n\
	.footer-sidebar .widget .widget-title,\r\n\
	.primary-sidebar .widget .widget-title {\r\n\
		font-size: 11px;\r\n\
		font-weight: 900;\r\n\
		line-height: 1.6363636363;\r\n\
		margin-bottom: 18px;\r\n\
	}\r\n\
\r\n\
	.footer-sidebar .widget_twentyfourteen_ephemera .entry-title,\r\n\
	.footer-sidebar .widget_twentyfourteen_ephemera .entry-meta,\r\n\
	.footer-sidebar .widget_twentyfourteen_ephemera .wp-caption-text,\r\n\
	.footer-sidebar .widget_twentyfourteen_ephemera .post-format-archive-link,\r\n\
	.footer-sidebar .widget_twentyfourteen_ephemera .entry-content table,\r\n\
	.primary-sidebar .widget_twentyfourteen_ephemera .entry-title,\r\n\
	.primary-sidebar .widget_twentyfourteen_ephemera .entry-meta,\r\n\
	.primary-sidebar .widget_twentyfourteen_ephemera .wp-caption-text,\r\n\
	.primary-sidebar .widget_twentyfourteen_ephemera .post-format-archive-link,\r\n\
	.primary-sidebar .widget_twentyfourteen_ephemera .entry-content table {\r\n\
		font-size: 11px;\r\n\
		line-height: 1.6363636363;\r\n\
	}\r\n\
\r\n\
	.footer-sidebar .widget_archive li,\r\n\
	.footer-sidebar .widget_categories li,\r\n\
	.footer-sidebar .widget_links li,\r\n\
	.footer-sidebar .widget_meta li,\r\n\
	.footer-sidebar .widget_nav_menu li,\r\n\
	.footer-sidebar .widget_pages li,\r\n\
	.footer-sidebar .widget_recent_comments li,\r\n\
	.footer-sidebar .widget_recent_entries li,\r\n\
	.primary-sidebar .widget_archive li,\r\n\
	.primary-sidebar .widget_categories li,\r\n\
	.primary-sidebar .widget_links li,\r\n\
	.primary-sidebar .widget_meta li,\r\n\
	.primary-sidebar .widget_nav_menu li,\r\n\
	.primary-sidebar .widget_pages li,\r\n\
	.primary-sidebar .widget_recent_comments li,\r\n\
	.primary-sidebar .widget_recent_entries li {\r\n\
		border-top: 0;\r\n\
		padding: 0 0 6px;\r\n\
	}\r\n\
\r\n\
	.footer-sidebar .widget_archive li:last-child,\r\n\
	.footer-sidebar .widget_categories li:last-child,\r\n\
	.footer-sidebar .widget_links li:last-child,\r\n\
	.footer-sidebar .widget_meta li:last-child,\r\n\
	.footer-sidebar .widget_nav_menu li:last-child,\r\n\
	.footer-sidebar .widget_pages li:last-child,\r\n\
	.footer-sidebar .widget_recent_comments li:last-child,\r\n\
	.footer-sidebar .widget_recent_entries li:last-child,\r\n\
	.primary-sidebar .widget_archive li:last-child,\r\n\
	.primary-sidebar .widget_categories li:last-child,\r\n\
	.primary-sidebar .widget_links li:last-child,\r\n\
	.primary-sidebar .widget_meta li:last-child,\r\n\
	.primary-sidebar .widget_nav_menu li:last-child,\r\n\
	.primary-sidebar .widget_pages li:last-child,\r\n\
	.primary-sidebar .widget_recent_comments li:last-child,\r\n\
	.primary-sidebar .widget_recent_entries li:last-child {\r\n\
		padding: 0;\r\n\
	}\r\n\
\r\n\
	.footer-sidebar .widget_categories li ul,\r\n\
	.footer-sidebar .widget_nav_menu li ul,\r\n\
	.footer-sidebar .widget_pages li ul,\r\n\
	.primary-sidebar .widget_categories li ul,\r\n\
	.primary-sidebar .widget_nav_menu li ul,\r\n\
	.primary-sidebar .widget_pages li ul {\r\n\
		border-top: 0;\r\n\
		margin-top: 6px;\r\n\
	}\r\n\
\r\n\
	#supplementary {\r\n\
		padding: 0;\r\n\
	}\r\n\
\r\n\
	.footer-sidebar {\r\n\
		font-size: 12px;\r\n\
		line-height: 1.5;\r\n\
	}\r\n\
\r\n\
	.featured-content {\r\n\
		padding-left: 182px;\r\n\
	}\r\n\
\r\n\
	.grid .featured-content .hentry {\r\n\
		width: 33.3333333%;\r\n\
	}\r\n\
\r\n\
	.grid .featured-content .hentry:nth-child( 2n+1 ) {\r\n\
		clear: none;\r\n\
	}\r\n\
\r\n\
	.grid .featured-content .hentry:nth-child( 3n+1 ) {\r\n\
		clear: both;\r\n\
	}\r\n\
\r\n\
	.grid .featured-content .entry-header {\r\n\
		height: 120px;\r\n\
	}\r\n\
}\r\n\
\r\n\
@media screen and (min-width: 1040px) {\r\n\
	.site-content .has-post-thumbnail .entry-header {\r\n\
		margin-top: -48px;\r\n\
	}\r\n\
\r\n\
	.archive-header,\r\n\
	.comments-area,\r\n\
	.image-navigation,\r\n\
	.page-header,\r\n\
	.page-content,\r\n\
	.post-navigation,\r\n\
	.site-content .entry-header,\r\n\
	.site-content .entry-content,\r\n\
	.site-content .entry-summary,\r\n\
	.site-content footer.entry-meta {\r\n\
		padding-right: 15px;\r\n\
		padding-left: 15px;\r\n\
	}\r\n\
\r\n\
	.full-width .archive-header,\r\n\
	.full-width .comments-area,\r\n\
	.full-width .image-navigation,\r\n\
	.full-width .page-header,\r\n\
	.full-width .page-content,\r\n\
	.full-width .post-navigation,\r\n\
	.full-width .site-content .entry-header,\r\n\
	.full-width .site-content .entry-content,\r\n\
	.full-width .site-content .entry-summary,\r\n\
	.full-width .site-content footer.entry-meta {\r\n\
		padding-right: 30px;\r\n\
		padding-left: 30px;\r\n\
	}\r\n\
}\r\n\
\r\n\
@media screen and (min-width: 1080px) {\r\n\
	.search-box .search-field {\r\n\
		width: 324px;\r\n\
	}\r\n\
\r\n\
	.site-content,\r\n\
	.site-main .widecolumn {\r\n\
		margin-left: 222px;\r\n\
	}\r\n\
\r\n\
	.site:before {\r\n\
		width: 222px;\r\n\
	}\r\n\
\r\n\
	.search-box-wrapper,\r\n\
	.featured-content {\r\n\
		padding-left: 222px;\r\n\
	}\r\n\
\r\n\
	#secondary {\r\n\
		width: 162px;\r\n\
	}\r\n\
\r\n\
	.secondary-navigation,\r\n\
	.secondary-navigation ul ul {\r\n\
		width: 222px;\r\n\
	}\r\n\
\r\n\
	.secondary-navigation ul li:hover > ul,\r\n\
	.secondary-navigation ul li.focus > ul {\r\n\
		left: 202px;\r\n\
	}\r\n\
\r\n\
	.slider .featured-content .entry-title {\r\n\
		font-size: 33px;\r\n\
	}\r\n\
\r\n\
	.slider .featured-content .entry-header,\r\n\
	.slider-control-paging {\r\n\
		width: 534px;\r\n\
	}\r\n\
\r\n\
	.slider-control-paging {\r\n\
		padding-left: 24px;\r\n\
	}\r\n\
\r\n\
	.slider-control-paging li {\r\n\
		margin: 12px 12px 12px 0;\r\n\
	}\r\n\
\r\n\
	.slider-control-paging a {\r\n\
		height: 24px;\r\n\
		width: 24px;\r\n\
	}\r\n\
\r\n\
	.slider-control-paging a:before {\r\n\
		top: 6px;\r\n\
		left: 6px;\r\n\
	}\r\n\
}\r\n\
\r\n\
@media screen and (min-width: 1110px) {\r\n\
	.archive-header,\r\n\
	.comments-area,\r\n\
	.image-navigation,\r\n\
	.page-header,\r\n\
	.page-content,\r\n\
	.post-navigation,\r\n\
	.site-content .entry-header,\r\n\
	.site-content .entry-content,\r\n\
	.site-content .entry-summary,\r\n\
	.site-content footer.entry-meta {\r\n\
		padding-right: 30px;\r\n\
		padding-left: 30px;\r\n\
	}\r\n\
}\r\n\
\r\n\
@media screen and (min-width: 1218px) {\r\n\
	.archive-header,\r\n\
	.comments-area,\r\n\
	.image-navigation,\r\n\
	.page-header,\r\n\
	.page-content,\r\n\
	.post-navigation,\r\n\
	.site-content .entry-header,\r\n\
	.site-content .entry-content,\r\n\
	.site-content .entry-summary,\r\n\
	.site-content footer.entry-meta {\r\n\
		margin-right: 54px;\r\n\
	}\r\n\
\r\n\
	.full-width .archive-header,\r\n\
	.full-width .comments-area,\r\n\
	.full-width .image-navigation,\r\n\
	.full-width .page-header,\r\n\
	.full-width .page-content,\r\n\
	.full-width .post-navigation,\r\n\
	.full-width .site-content .entry-header,\r\n\
	.full-width .site-content .entry-content,\r\n\
	.full-width .site-content .entry-summary,\r\n\
	.full-width .site-content footer.entry-meta {\r\n\
		margin-right: auto;\r\n\
	}\r\n\
}\r\n\
\r\n\
@media screen and (min-width: 1260px) {\r\n\
	.site-content blockquote.alignleft,\r\n\
	.site-content blockquote.alignright {\r\n\
		width: -webkit-calc(50% + 18px);\r\n\
		width:         calc(50% + 18px);\r\n\
	}\r\n\
\r\n\
	.site-content blockquote.alignleft {\r\n\
		margin-left: -18%;\r\n\
	}\r\n\
\r\n\
	.site-content blockquote.alignright {\r\n\
		margin-right: -18%;\r\n\
	}\r\n\
}\r\n\
\r\n\
\r\n\
/**\r\n\
 * 12.0 Print\r\n\
 * -----------------------------------------------------------------------------\r\n\
 */\r\n\
\r\n\
@media print {\r\n\
	body {\r\n\
		background: none !important; /* Brute force since user agents all print differently. */\r\n\
		color: #2b2b2b;\r\n\
		font-size: 12pt;\r\n\
	}\r\n\
\r\n\
	.site,\r\n\
	.site-header,\r\n\
	.hentry,\r\n\
	.site-content .entry-header,\r\n\
	.site-content .entry-content,\r\n\
	.site-content .entry-summary,\r\n\
	.site-content .entry-meta,\r\n\
	.page-content,\r\n\
	.archive-header,\r\n\
	.page-header,\r\n\
	.contributor-info,\r\n\
	.comments-area,\r\n\
	.attachment .entry-attachment .attachment {\r\n\
		max-width: 100%;\r\n\
	}\r\n\
\r\n\
	#site-header img,\r\n\
	.search-toggle,\r\n\
	.site-navigation,\r\n\
	.site-content nav,\r\n\
	.edit-link,\r\n\
	.page-links,\r\n\
	.widget-area,\r\n\
	.more-link,\r\n\
	.post-format-archive-link,\r\n\
	.comment-respond,\r\n\
	.comment-list .reply,\r\n\
	.comment-reply-login,\r\n\
	#secondary,\r\n\
	.site-footer,\r\n\
	.slider-control-paging,\r\n\
	.slider-direction-nav {\r\n\
		display: none;\r\n\
	}\r\n\
\r\n\
	.site-title a,\r\n\
	.entry-meta,\r\n\
	.entry-meta a,\r\n\
	.featured-content .hentry,\r\n\
	.featured-content a {\r\n\
		color: #2b2b2b;\r\n\
	}\r\n\
\r\n\
	.entry-content a,\r\n\
	.entry-summary a,\r\n\
	.page-content a,\r\n\
	.comment-content a {\r\n\
		text-decoration: none;\r\n\
	}\r\n\
\r\n\
	.site-header,\r\n\
	.post-thumbnail,\r\n\
	a.post-thumbnail:hover,\r\n\
	.site-content .entry-header,\r\n\
	.site-footer,\r\n\
	.featured-content,\r\n\
	.featured-content .entry-header {\r\n\
		background: transparent;\r\n\
	}\r\n\
\r\n\
	.header-main {\r\n\
		padding: 48px 10px;\r\n\
	}\r\n\
\r\n\
	.site-title {\r\n\
		float: none;\r\n\
		font-size: 19pt;\r\n\
	}\r\n\
\r\n\
	.content-area {\r\n\
		padding-top: 0;\r\n\
	}\r\n\
\r\n\
	.list-view .site-content .hentry {\r\n\
		border-bottom: 1px solid rgba(0, 0, 0, 0.1);\r\n\
		margin-bottom: 48px;\r\n\
		padding-bottom: 24px;\r\n\
	}\r\n\
\r\n\
	.post-thumbnail img {\r\n\
		margin: 0 10px 24px;\r\n\
	}\r\n\
\r\n\
	.site-content .has-post-thumbnail .entry-header {\r\n\
		padding-top: 0;\r\n\
	}\r\n\
\r\n\
	.site-content footer.entry-meta {\r\n\
		margin: 24px auto;\r\n\
	}\r\n\
\r\n\
	.entry-meta .tag-links a {\r\n\
		color: #fff;\r\n\
	}\r\n\
\r\n\
	.singular .site-content .hentry.has-post-thumbnail {\r\n\
		margin-top: 0;\r\n\
	}\r\n\
\r\n\
	.gallery-columns-1.gallery-size-medium,\r\n\
	.gallery-columns-1.gallery-size-thumbnail,\r\n\
	.gallery-columns-2.gallery-size-thumbnail,\r\n\
	.gallery-columns-3.gallery-size-thumbnail {\r\n\
		display: block;\r\n\
	}\r\n\
\r\n\
	.archive-title,\r\n\
	.page-title {\r\n\
		margin: 0 10px 48px;\r\n\
	}\r\n\
\r\n\
	.featured-content .hentry {\r\n\
		margin-bottom: 48px;\r\n\
	}\r\n\
\r\n\
	.featured-content .post-thumbnail,\r\n\
	.slider .featured-content .post-thumbnail {\r\n\
		padding-top: 0;\r\n\
	}\r\n\
\r\n\
	.featured-content .post-thumbnail img {\r\n\
		position: relative;\r\n\
	}\r\n\
\r\n\
	.featured-content .entry-header {\r\n\
		padding: 0 10px 24px;\r\n\
	}\r\n\
\r\n\
	.featured-content .entry-meta {\r\n\
		font-size: 9pt;\r\n\
		margin-bottom: 11px;\r\n\
	}\r\n\
\r\n\
	.featured-content .cat-links {\r\n\
		font-weight: 900;\r\n\
	}\r\n\
\r\n\
	.featured-content .entry-title {\r\n\
		font-size: 25pt;\r\n\
		line-height: 36px;\r\n\
	}\r\n\
}\r\n\
</style>\r\n\
\r\n\
<meta name="generator" content="WordPress 4.5.3">\r\n\
		<style type="text/css">.recentcomments a{display:inline !important;padding:0 !important;margin:0 !important;}</style>\r\n\
<style type="text/css">.MathJax_Hover_Frame {border-radius: .25em; -webkit-border-radius: .25em; -moz-border-radius: .25em; -khtml-border-radius: .25em; box-shadow: 0px 0px 15px #83A; -webkit-box-shadow: 0px 0px 15px #83A; -moz-box-shadow: 0px 0px 15px #83A; -khtml-box-shadow: 0px 0px 15px #83A; border: 1px solid #A6D ! important; display: inline-block; position: absolute}\r\n\
.MathJax_Menu_Button .MathJax_Hover_Arrow {position: absolute; cursor: pointer; display: inline-block; border: 2px solid #AAA; border-radius: 4px; -webkit-border-radius: 4px; -moz-border-radius: 4px; -khtml-border-radius: 4px; font-family: \'Courier New\',Courier; font-size: 9px; color: #F0F0F0}\r\n\
.MathJax_Menu_Button .MathJax_Hover_Arrow span {display: block; background-color: #AAA; border: 1px solid; border-radius: 3px; line-height: 0; padding: 4px}\r\n\
.MathJax_Hover_Arrow:hover {color: white!important; border: 2px solid #CCC!important}\r\n\
.MathJax_Hover_Arrow:hover span {background-color: #CCC!important}\r\n\
</style><style type="text/css">#MathJax_About {position: fixed; left: 50%; width: auto; text-align: center; border: 3px outset; padding: 1em 2em; background-color: #DDDDDD; color: black; cursor: default; font-family: message-box; font-size: 120%; font-style: normal; text-indent: 0; text-transform: none; line-height: normal; letter-spacing: normal; word-spacing: normal; word-wrap: normal; white-space: nowrap; float: none; z-index: 201; border-radius: 15px; -webkit-border-radius: 15px; -moz-border-radius: 15px; -khtml-border-radius: 15px; box-shadow: 0px 10px 20px #808080; -webkit-box-shadow: 0px 10px 20px #808080; -moz-box-shadow: 0px 10px 20px #808080; -khtml-box-shadow: 0px 10px 20px #808080; filter: progid:DXImageTransform.Microsoft.dropshadow(OffX=2, OffY=2, Color=\'gray\', Positive=\'true\')}\r\n\
#MathJax_About.MathJax_MousePost {outline: none}\r\n\
.MathJax_Menu {position: absolute; background-color: white; color: black; width: auto; padding: 2px; border: 1px solid #CCCCCC; margin: 0; cursor: default; font: menu; text-align: left; text-indent: 0; text-transform: none; line-height: normal; letter-spacing: normal; word-spacing: normal; word-wrap: normal; white-space: nowrap; float: none; z-index: 201; box-shadow: 0px 10px 20px #808080; -webkit-box-shadow: 0px 10px 20px #808080; -moz-box-shadow: 0px 10px 20px #808080; -khtml-box-shadow: 0px 10px 20px #808080; filter: progid:DXImageTransform.Microsoft.dropshadow(OffX=2, OffY=2, Color=\'gray\', Positive=\'true\')}\r\n\
.MathJax_MenuItem {padding: 2px 2em; background: transparent}\r\n\
.MathJax_MenuArrow {position: absolute; right: .5em; padding-top: .25em; color: #666666; font-size: .75em}\r\n\
.MathJax_MenuActive .MathJax_MenuArrow {color: white}\r\n\
.MathJax_MenuArrow.RTL {left: .5em; right: auto}\r\n\
.MathJax_MenuCheck {position: absolute; left: .7em}\r\n\
.MathJax_MenuCheck.RTL {right: .7em; left: auto}\r\n\
.MathJax_MenuRadioCheck {position: absolute; left: 1em}\r\n\
.MathJax_MenuRadioCheck.RTL {right: 1em; left: auto}\r\n\
.MathJax_MenuLabel {padding: 2px 2em 4px 1.33em; font-style: italic}\r\n\
.MathJax_MenuRule {border-top: 1px solid #CCCCCC; margin: 4px 1px 0px}\r\n\
.MathJax_MenuDisabled {color: GrayText}\r\n\
.MathJax_MenuActive {background-color: Highlight; color: HighlightText}\r\n\
.MathJax_MenuDisabled:focus, .MathJax_MenuLabel:focus {background-color: #E8E8E8}\r\n\
.MathJax_ContextMenu:focus {outline: none}\r\n\
.MathJax_ContextMenu .MathJax_MenuItem:focus {outline: none}\r\n\
#MathJax_AboutClose {top: .2em; right: .2em}\r\n\
.MathJax_Menu .MathJax_MenuClose {top: -10px; left: -10px}\r\n\
.MathJax_MenuClose {position: absolute; cursor: pointer; display: inline-block; border: 2px solid #AAA; border-radius: 18px; -webkit-border-radius: 18px; -moz-border-radius: 18px; -khtml-border-radius: 18px; font-family: \'Courier New\',Courier; font-size: 24px; color: #F0F0F0}\r\n\
.MathJax_MenuClose span {display: block; background-color: #AAA; border: 1.5px solid; border-radius: 18px; -webkit-border-radius: 18px; -moz-border-radius: 18px; -khtml-border-radius: 18px; line-height: 0; padding: 8px 0 6px}\r\n\
.MathJax_MenuClose:hover {color: white!important; border: 2px solid #CCC!important}\r\n\
.MathJax_MenuClose:hover span {background-color: #CCC!important}\r\n\
.MathJax_MenuClose:hover:focus {outline: none}\r\n\
</style><style type="text/css">.MathJax_Preview .MJXf-math {color: inherit!important}\r\n\
</style><style type="text/css">.MJX_Assistive_MathML {position: absolute!important; top: 0; left: 0; clip: rect(1px, 1px, 1px, 1px); padding: 1px 0 0 0!important; border: 0!important; height: 1px!important; width: 1px!important; overflow: hidden!important; display: block!important; -webkit-touch-callout: none; -webkit-user-select: none; -khtml-user-select: none; -moz-user-select: none; -ms-user-select: none; user-select: none}\r\n\
.MJX_Assistive_MathML.MJX_Assistive_MathML_Block {width: 100%!important}\r\n\
</style><style type="text/css">#MathJax_Zoom {position: absolute; background-color: #F0F0F0; overflow: auto; display: block; z-index: 301; padding: .5em; border: 1px solid black; margin: 0; font-weight: normal; font-style: normal; text-align: left; text-indent: 0; text-transform: none; line-height: normal; letter-spacing: normal; word-spacing: normal; word-wrap: normal; white-space: nowrap; float: none; -webkit-box-sizing: content-box; -moz-box-sizing: content-box; box-sizing: content-box; box-shadow: 5px 5px 15px #AAAAAA; -webkit-box-shadow: 5px 5px 15px #AAAAAA; -moz-box-shadow: 5px 5px 15px #AAAAAA; -khtml-box-shadow: 5px 5px 15px #AAAAAA; filter: progid:DXImageTransform.Microsoft.dropshadow(OffX=2, OffY=2, Color=\'gray\', Positive=\'true\')}\r\n\
#MathJax_ZoomOverlay {position: absolute; left: 0; top: 0; z-index: 300; display: inline-block; width: 100%; height: 100%; border: 0; padding: 0; margin: 0; background-color: white; opacity: 0; filter: alpha(opacity=0)}\r\n\
#MathJax_ZoomFrame {position: relative; display: inline-block; height: 0; width: 0}\r\n\
#MathJax_ZoomEventTrap {position: absolute; left: 0; top: 0; z-index: 302; display: inline-block; border: 0; padding: 0; margin: 0; background-color: white; opacity: 0; filter: alpha(opacity=0)}\r\n\
</style><style type="text/css">.MathJax_Preview {color: #888}\r\n\
#MathJax_Message {position: fixed; left: 1px; bottom: 2px; background-color: #E6E6E6; border: 1px solid #959595; margin: 0px; padding: 2px 8px; z-index: 102; color: black; font-size: 80%; width: auto; white-space: nowrap}\r\n\
#MathJax_MSIE_Frame {position: absolute; top: 0; left: 0; width: 0px; z-index: 101; border: 0px; margin: 0px; padding: 0px}\r\n\
.MathJax_Error {color: #CC0000; font-style: italic}\r\n\
</style><style type="text/css">.MJXp-script {font-size: .8em}\r\n\
.MJXp-right {-webkit-transform-origin: right; -moz-transform-origin: right; -ms-transform-origin: right; -o-transform-origin: right; transform-origin: right}\r\n\
.MJXp-bold {font-weight: bold}\r\n\
.MJXp-italic {font-style: italic}\r\n\
.MJXp-scr {font-family: MathJax_Script,\'Times New Roman\',Times,STIXGeneral,serif}\r\n\
.MJXp-frak {font-family: MathJax_Fraktur,\'Times New Roman\',Times,STIXGeneral,serif}\r\n\
.MJXp-sf {font-family: MathJax_SansSerif,\'Times New Roman\',Times,STIXGeneral,serif}\r\n\
.MJXp-cal {font-family: MathJax_Caligraphic,\'Times New Roman\',Times,STIXGeneral,serif}\r\n\
.MJXp-mono {font-family: MathJax_Typewriter,\'Times New Roman\',Times,STIXGeneral,serif}\r\n\
.MJXp-largeop {font-size: 150%}\r\n\
.MJXp-largeop.MJXp-int {vertical-align: -.2em}\r\n\
.MJXp-math {display: inline-block; line-height: 1.2; text-indent: 0; font-family: \'Times New Roman\',Times,STIXGeneral,serif; white-space: nowrap; border-collapse: collapse}\r\n\
.MJXp-display {display: block; text-align: center; margin: 1em 0}\r\n\
.MJXp-math span {display: inline-block}\r\n\
.MJXp-box {display: block!important; text-align: center}\r\n\
.MJXp-box:after {content: " "}\r\n\
.MJXp-rule {display: block!important; margin-top: .1em}\r\n\
.MJXp-char {display: block!important}\r\n\
.MJXp-mo {margin: 0 .15em}\r\n\
.MJXp-mfrac {margin: 0 .125em; vertical-align: .25em}\r\n\
.MJXp-denom {display: inline-table!important; width: 100%}\r\n\
.MJXp-denom > * {display: table-row!important}\r\n\
.MJXp-surd {vertical-align: top}\r\n\
.MJXp-surd > * {display: block!important}\r\n\
.MJXp-script-box > *  {display: table!important; height: 50%}\r\n\
.MJXp-script-box > * > * {display: table-cell!important; vertical-align: top}\r\n\
.MJXp-script-box > *:last-child > * {vertical-align: bottom}\r\n\
.MJXp-script-box > * > * > * {display: block!important}\r\n\
.MJXp-mphantom {visibility: hidden}\r\n\
.MJXp-munderover {display: inline-table!important}\r\n\
.MJXp-over {display: inline-block!important; text-align: center}\r\n\
.MJXp-over > * {display: block!important}\r\n\
.MJXp-munderover > * {display: table-row!important}\r\n\
.MJXp-mtable {vertical-align: .25em; margin: 0 .125em}\r\n\
.MJXp-mtable > * {display: inline-table!important; vertical-align: middle}\r\n\
.MJXp-mtr {display: table-row!important}\r\n\
.MJXp-mtd {display: table-cell!important; text-align: center; padding: .5em 0 0 .5em}\r\n\
.MJXp-mtr > .MJXp-mtd:first-child {padding-left: 0}\r\n\
.MJXp-mtr:first-child > .MJXp-mtd {padding-top: 0}\r\n\
.MJXp-mlabeledtr {display: table-row!important}\r\n\
.MJXp-mlabeledtr > .MJXp-mtd:first-child {padding-left: 0}\r\n\
.MJXp-mlabeledtr:first-child > .MJXp-mtd {padding-top: 0}\r\n\
.MJXp-merror {background-color: #FFFF88; color: #CC0000; border: 1px solid #CC0000; padding: 1px 3px; font-style: normal; font-size: 90%}\r\n\
.MJXp-scale0 {-webkit-transform: scaleX(.0); -moz-transform: scaleX(.0); -ms-transform: scaleX(.0); -o-transform: scaleX(.0); transform: scaleX(.0)}\r\n\
.MJXp-scale1 {-webkit-transform: scaleX(.1); -moz-transform: scaleX(.1); -ms-transform: scaleX(.1); -o-transform: scaleX(.1); transform: scaleX(.1)}\r\n\
.MJXp-scale2 {-webkit-transform: scaleX(.2); -moz-transform: scaleX(.2); -ms-transform: scaleX(.2); -o-transform: scaleX(.2); transform: scaleX(.2)}\r\n\
.MJXp-scale3 {-webkit-transform: scaleX(.3); -moz-transform: scaleX(.3); -ms-transform: scaleX(.3); -o-transform: scaleX(.3); transform: scaleX(.3)}\r\n\
.MJXp-scale4 {-webkit-transform: scaleX(.4); -moz-transform: scaleX(.4); -ms-transform: scaleX(.4); -o-transform: scaleX(.4); transform: scaleX(.4)}\r\n\
.MJXp-scale5 {-webkit-transform: scaleX(.5); -moz-transform: scaleX(.5); -ms-transform: scaleX(.5); -o-transform: scaleX(.5); transform: scaleX(.5)}\r\n\
.MJXp-scale6 {-webkit-transform: scaleX(.6); -moz-transform: scaleX(.6); -ms-transform: scaleX(.6); -o-transform: scaleX(.6); transform: scaleX(.6)}\r\n\
.MJXp-scale7 {-webkit-transform: scaleX(.7); -moz-transform: scaleX(.7); -ms-transform: scaleX(.7); -o-transform: scaleX(.7); transform: scaleX(.7)}\r\n\
.MJXp-scale8 {-webkit-transform: scaleX(.8); -moz-transform: scaleX(.8); -ms-transform: scaleX(.8); -o-transform: scaleX(.8); transform: scaleX(.8)}\r\n\
.MJXp-scale9 {-webkit-transform: scaleX(.9); -moz-transform: scaleX(.9); -ms-transform: scaleX(.9); -o-transform: scaleX(.9); transform: scaleX(.9)}\r\n\
.MathJax_PHTML .noError {vertical-align: ; font-size: 90%; text-align: left; color: black; padding: 1px 3px; border: 1px solid}\r\n\
</style>\
';

// This code is placed at the beginning of the body before the Markdeep code. 
// $ (DOCUMENT_BODY_PREFIX) is everything in the body of PreviewBlogPage.htm up to 
// $ (ARTICLE_HTML_CODE).
DocumentBodyPrefix='\
<div style="display: none;" id="MathJax_Message"></div>\r\n\
<div id="page" class="hfeed site">\r\n\
	\r\n\
	<header id="masthead" class="site-header" role="banner">\r\n\
		<div class="header-main">\r\n\
			<h1 class="site-title">Preview blog</h1>\r\n\
\r\n\
			<div class="search-toggle">\r\n\
				<a href="#search-container" class="screen-reader-text" aria-expanded="false" aria-controls="search-container">Search</a>\r\n\
			</div>\r\n\
\r\n\
			<nav id="primary-navigation" class="site-navigation primary-navigation" role="navigation">\r\n\
				<button class="menu-toggle">Primary Menu</button>\r\n\
				<a class="screen-reader-text skip-link" href="#content">Skip to content</a>\r\n\
				<div id="primary-menu" class="nav-menu"><ul></ul></div>\r\n\
			</nav>\r\n\
		</div>\r\n\
\r\n\
		<div id="search-container" class="search-box-wrapper hide">\r\n\
			<div class="search-box">\r\n\
				<form role="search" method="get" class="search-form">\r\n\
				<label>\r\n\
					<span class="screen-reader-text">Search for:</span>\r\n\
					<input class="search-field" placeholder="Search" name="s" type="search">\r\n\
				</label>\r\n\
				<input class="search-submit" value="Search" type="submit">\r\n\
			</form>			</div>\r\n\
		</div>\r\n\
	</header><!-- #masthead -->\r\n\
\r\n\
	<div id="main" class="site-main">\r\n\
\r\n\
<div id="main-content" class="main-content">\r\n\
\r\n\
	<div id="primary" class="content-area">\r\n\
		<div id="content" class="site-content" role="main">\r\n\
\r\n\
			\r\n\
<article id="post-14" class="post-14 page type-page status-publish hentry">\r\n\
\r\n\
	<header class="entry-header"><h1 class="entry-title">Blog post preview</h1></header><!-- .entry-header -->\r\n\
	<div class="entry-content">\r\n\
		<div><span class="md">\r\n\
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
\r\n\
		</span></div>\r\n\
	</div><!-- .entry-content -->\r\n\
</article><!-- #post-## -->\r\n\
\r\n\
		</div><!-- #content -->\r\n\
	</div><!-- #primary -->\r\n\
	</div><!-- #main-content -->\r\n\
\r\n\
<div id="secondary">\r\n\
		<h2 class="site-description">Not a live webpage</h2>\r\n\
	\r\n\
	\r\n\
		<div id="primary-sidebar" class="primary-sidebar widget-area" role="complementary">\r\n\
		<aside id="search-2" class="widget widget_search"><form role="search" method="get" class="search-form" action="http://momentsingraphics.de/">\r\n\
				<label>\r\n\
					<span class="screen-reader-text">Search for:</span>\r\n\
					<input class="search-field" placeholder="Search" name="s" type="search">\r\n\
				</label>\r\n\
				<input class="search-submit" value="Search" type="submit">\r\n\
			</form></aside>		<aside id="recent-posts-2" class="widget widget_recent_entries">		<h1 class="widget-title">Recent Posts</h1>		<ul>\r\n\
					<li>\r\n\
						Blog post preview\r\n\
					</li>\r\n\
				</ul>\r\n\
		</aside>		<aside id="recent-comments-2" class="widget widget_recent_comments"><h1 class="widget-title">Recent Comments</h1><ul id="recentcomments"></ul></aside><aside id="archives-2" class="widget widget_archive"><h1 class="widget-title">Archives</h1>\r\n\
		</aside><aside id="meta-2" class="widget widget_meta"><h1 class="widget-title">Meta</h1>			<ul>\r\n\
						<li>Log in</li>\r\n\
			<li>Entries <abbr title="Really Simple Syndication">RSS</abbr></li>\r\n\
			<li>Comments <abbr title="Really Simple Syndication">RSS</abbr></li>\r\n\
			<li><a href="https://wordpress.org/" title="Powered by WordPress, state-of-the-art semantic personal publishing platform.">WordPress.org</a></li>			</ul>\r\n\
			</aside></div><!-- #primary-sidebar -->\r\n\
	</div><!-- #secondary -->\r\n\
\r\n\
		</div><!-- #main -->\r\n\
\r\n\
		<footer id="colophon" class="site-footer" role="contentinfo">\r\n\
\r\n\
			\r\n\
			<div class="site-info">\r\n\
								Powered by <a href="https://wordpress.org/">WordPress</a> and <a href="https://casual-effects.com/markdeep/">Markdeep</a>.\r\n\
			</div><!-- .site-info -->\r\n\
		</footer><!-- #colophon -->\r\n\
	</div><!-- #page -->\r\n\
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
document.body.setAttribute("class","page page-id-14 page-template-default masthead-fixed full-width singular");


// SetMarkdeepMode.js

window.markdeepOptions={mode:"html"};


// markdeep.min.js

/**See http://casual-effects.com/markdeep for @license and documentation.
markdeep.min.js 0.17 (C) 2016 Morgan McGuire 
highlight.min.js 9.5.0 (C) 2016 Ivan Sagalaev https://highlightjs.org/*/
!function(){"use strict";function e(e,t,r){return"<"+e+(r?" "+r:"")+">"+t+"</"+e+">"}function t(e){try{var t=document.createElement("canvas"),r=t.getContext("2d");return r.font="10pt "+e,r.measureText("M").width}catch(e){return 10}}function r(e){return window.markdeepOptions&&void 0!==window.markdeepOptions[e]?window.markdeepOptions[e]:void 0!==Z[e]?Z[e]:void console.warn('Illegal option: "'+e+'"')}function n(t,n){if(r("showURLs")){var a=" {\xa0"+t+"\xa0}";return n?e(n,a):a}return""}function a(e){return r("lang").keyword[e]||e}function i(e){return(e+"").rp(/&/g,"&amp;").rp(/</g,"&lt;").rp(/>/g,"&gt;").rp(/"/g,"&quot;")}function o(e){return e.rp(/&lt;/g,"<").rp(/&gt;/g,">").rp(/&quot;/g,'"').rp(/&#39;/g,"'").rp(/&ndash;/g,"\u2013").rp(/&mdash;/g,"---").rp(/&amp;/g,"&")}function s(e){return e.rp(/<.*?>/g,"")}function c(e){return encodeURI(e.rp(/\s/g,"").toLowerCase())}function l(){for(var t="",r=1;r<=6;++r){t+="h"+r+"::before {\ncontent:";for(var n=1;n<=r;++n)t+="counter(h"+n+') "'+(n<r?".":" ")+'"';t+=";\ncounter-increment: h"+r+";margin-right:10px}"}return e("style",t)}function u(e,t){var r=e.innerHTML;return r=r.rp(/(?:<style class="fallback">[\s\S]*?<\/style>[\s\S]*)<\/\S+@\S+\.\S+?>/gim,""),r=r.rp(/<\/h?ttps?:.*>/gi,""),r=r.rp(/<(https?): (.*?)>/gi,function(e,t,r){var n="<"+t+"://"+r.rp(/=""\s/g,"/");return'=""'===n.ss(n.length-3)&&(n=n.ss(0,n.length-3)),n=n.rp(/"/g,""),n+">"}),r=r.rp(/<style class=["']fallback["']>.*?<\/style>/gim,""),r=o(r)}function d(e){function t(){l=e.indexOf("\n",i)+1,u=u||/\S/.test(e.ss(i,i+o)),d=d||/\S/.test(e.ss(i+s+1,l))}for(var r={p:e,q:"",s:"",u:""},n=e.indexOf($);n>=0;n=e.indexOf($,n+$.length)){var a,i=J(0,e.lastIndexOf("\n",n))+1,o=n-i;for(a=n+$.length;e[a]===I;++a);var s=a-i-1,c={p:e.ss(0,i),q:"",s:"center",u:e.ss(i,n).rp(/[ \t]+$/," ")},l=0,u=!1,d=!1;t();for(var g=!0,p=a;g;){if(i=l,t(),0===i)return r;if(u?c.s="floatright":d&&(c.s="floatleft"),e[i+o]!==I||u&&e[i+s]!==I)g=!1;else{var m;for(m=o;m<s&&e[i+m]===I;++m);var f=i+o,h=i+s;if(!u){var b=e.indexOf("\n",f);b!==-1&&(h=Math.min(h,b))}if(c.u+=e.ss(p,f).rp(/^[ \t]*[ \t]/," ").rp(/[ \t][ \t]*$/," "),m===s)return c.u+=e.ss(i+s+1),c;c.q+=e.ss(f+1,h)+"\n",p=h+1}}}return r}function g(e,t,r,n){var a=t.source,i="[^ \\t\\n"+a+"]",o="("+a+")("+i+".*?(\\n.+?)*?)"+a+"(?![A-Za-z0-9])";return e.rp(RegExp(o,"g"),"<"+r+(n?" "+n:"")+">$2</"+r+">")}function p(t,r){function n(e){return e.trim().rp(/^\||\|$/g,"")}var a=/(?:\n\|?[ \t\S]+?(?:\|[ \t\S]+?)+\|?(?=\n))/.source,i=/\n\|? *\:?-+\:?(?: *\| *\:?-+\:?)+ *\|?(?=\n)/.source,o=/\n[ \t]*\[[^\n\|]+\][ \t]*(?=\n)/.source,s=RegExp(a+i+a+"+("+o+")?","g");return t=t.rp(s,function(t){var a=t.split("\n"),i="",o=""===a[0]?1:0,s=a[a.length-1].trim();s.length>3&&"["===s[0]&&"]"===s[s.length-1]?(a.pop(),s=s.ss(1,s.length-1)):s=void 0;var c=[];n(a[o+1]).rp(/:?-+:?/g,function(e){var t=":"===e[0],n=":"===e[e.length-1];c.push(r(' style="text-align:'+(t&&n?"center":n?"right":"left")+'"'))});for(var l="th",u=o;u<a.length;++u){var d=n(a[u].trim()),g=0;i+=e("tr","<"+l+c[0]+">"+d.rp(/\|/g,function(){return++g,"</"+l+"><"+l+c[g]+">"})+"</"+l+">")+"\n",u==o&&(++u,l="td")}return i=e("table",i,r('class="table"')),s&&(i=e("div",s,r('class="tablecaption"'))+i),i})}function m(e,t){for(var r=/^\s*\n/.source,n=/[:,]\s*\n/.source,a=RegExp("("+n+"|"+r+")"+/((?:[ \t]*(?:\d+\.|-|\+|\*)(?:[ \t]+.+\n(?:[ \t]*\n)?)+)+)/.source,"gm"),i=!0,o={"+":t('class="plus"'),"-":t('class="minus"'),"*":t('class="asterisk"')},s=t('class="number"');i;)i=!1,e=e.rp(a,function(e,t,r){var n=t,a=[],c={F:-1};for(r.split("\n").forEach(function(e){var t=e.rp(/^\s*/,""),r=e.length-t.length,l=o[t[0]],u=!!l;l=l||s;var d=/^\d+\.[ \t]/.test(t);if(c)if(d||u){if(r!==c.F)if(c.F!==-1&&r<c.F)for(;c&&r<c.F;)a.pop(),n+="\n</li></"+c.tag+">",c=a[a.length-1];else c={F:r,tag:d?"ol":"ul",G:e.ss(0,r)},a.push(c),n+="\n<"+c.tag+">";else c.F!==-1&&(n+="\n</li>");c?n+="\n"+c.G+"<li "+l+">"+t.rp(/^(\d+\.|-|\+|\*) /,""):(n+="\n"+e,i=!0)}else n+="\n"+c.G+e;else n+="\n"+e}),c=a.pop();c;c=a.pop())n+="</li></"+c.tag+">\n";return n});return e}function f(t,n){var i=/^(?:[^\|<>\s-\+\*\d].*[12]\d{3}(?!\d).*?|(?:[12]\d{3}(?!\.).*\d.*?)|(?:\d{1,3}(?!\.).*[12]\d{3}(?!\d).*?))/.source,o="("+i+"):"+/[ \t]+([^ \t\n].*)\n/.source,s=/(?:[ \t]*\n)?((?:[ \t]+.+\n(?:[ \t]*\n){0,3})*)/.source,c=o+s,l=RegExp(c,"gm"),u=n('valign="top"'),d=n('style="width:100px;padding-right:15px" rowspan="2"'),g=n('style="padding-bottom:25px"'),p=["Sunday","Monday","Tuesday","Wednesday","Thursday","Friday","Saturday"].map(a),m=["jan","feb","mar","apr","may","jun","jul","aug","sep","oct","nov","dec"].map(a),f=m.join("|"),h=["January","February","March","April","May","June","July","August","September","October","November","December"].map(a),b=9;try{var x=0;t=t.rp(RegExp("("+c+"){2,}","gm"),function(t){++x;var a=[],i=!1;t.rp(l,function(t,r,o,s){var c="",l="",h="",y=!1;r=r.trim(),"("===r[0]&&")"===r.slice(-1)&&(r=r.slice(1,-1),y=!0);var v=r.match(RegExp("([0123]?\\d)\\D+([01]?\\d|"+f+")\\D+([12]\\d{3})","i"));if(v)h=v[1],l=v[2],c=v[3];else if(v=r.match(RegExp("([12]\\d{3})\\D+([01]?\\d|"+f+")\\D+([0123]?\\d)","i")))h=v[3],l=v[2],c=v[1];else{if(v=r.match(RegExp("("+f+")\\D+([0123]?\\d)\\D+([12]\\d{3})","i")),!v)throw"Could not parse date";h=v[2],l=v[1],c=v[3]}r=h+" "+l+" "+c;var _=parseInt(l)-1;isNaN(_)&&(_=m.indexOf(l.toLowerCase()));var w=new Date(Date.UTC(parseInt(c),_,parseInt(h),b)),C=w.getUTCDay();return r=p[C]+"<br/>"+r,i=i||0===C||6===C,a.push({date:w,title:o,sourceOrder:a.length,parenthesized:y,text:y?"":e("tr",e("td","<a "+n('name="schedule'+x+"_"+w.getUTCFullYear()+"-"+(w.getUTCMonth()+1)+"-"+w.getUTCDate()+'"')+"></a>"+r,d)+e("td",e("b",o)),u)+e("tr",e("td","\n\n"+s,g),u)}),""});var o=r("sortScheduleLists")?a:a.slice(0);a.sort(function(e,t){var r=e.date.getTime(),n=t.date.getTime();return r===n?e.sourceOrder-t.sourceOrder:r-n});var s=864e5,c=(a[a.length-1].date.getTime()-a[0].date.getTime())/s,y=new Date;y=new Date(Date.UTC(y.getUTCFullYear(),y.getUTCMonth(),y.getUTCDate(),b));var v="";if(c>14&&c/a.length<16){var _=n('colspan="2" width="14%" style="padding-top:5px;text-align:center;font-style:italic"'),w=n('width="1%" height="30px" style="text-align:right;border:1px solid #EEE;border-right:none;"'),C=n('width="1%" height="30px" style="color:#BBB;text-align:right;"'),M=n('width="14%" style="border:1px solid #EEE;border-left:none;"'),N=n('class="parenthesized"'),k=a[0].date,j=0,A=!i&&r("hideEmptyWeekends"),T=A?function(e){return e.getUTCDay()>0&&e.getUTCDay()<6}:function(){return!0},S=function(e,t){return Q(e.getTime()-t.getTime())<s/2};for(k=new Date(k.getUTCFullYear(),k.getUTCMonth(),1,b);k.getTime()<a[a.length-1].date.getTime();){for(v+="<table "+n('class="calendar"')+">\n"+e("tr",e("th",h[k.getUTCMonth()]+" "+k.getUTCFullYear(),n('colspan="14"')))+"<tr>",(A?p.slice(1,6):p).forEach(function(t){v+=e("td",t,_)}),v+="</tr>";0!==k.getUTCDay();)k=new Date(k.getTime()-s);if(1!==k.getDate())for(v+="<tr "+u+">";1!==k.getDate();)T(k)&&(v+="<td "+C+">"+k.getUTCDate()+"</td><td>&nbsp;</td>"),k=new Date(k.getTime()+s);do{if(0===k.getUTCDay()&&(v+="<tr "+u+">"),T(k)){var E="";S(k,y)&&(E=n('class="today"'));for(var B="",D=a[j];D&&S(D.date,k);++j,D=a[j])B&&(B+="<br/>"),B+=D.parenthesized?e("span",D.title,N):e("a",D.title,n('href="#schedule'+x+"_"+k.getUTCFullYear()+"-"+(k.getUTCMonth()+1)+"-"+k.getUTCDate()+'"'));v+=B?e("td",e("b",k.getUTCDate()),w+E)+e("td",B,M+E):"<td "+w+E+"></a>"+k.getUTCDate()+"</td><td "+M+E+"> &nbsp; </td>"}6===k.getUTCDay()&&(v+="</tr>"),k=new Date(k.getTime()+s)}while(k.getUTCDate()>1);if(0!==k.getUTCDay()){for(;0!==k.getUTCDay();)T(k)&&(v+="<td "+C+">"+k.getUTCDate()+"</td><td>&nbsp</td>"),k=new Date(k.getTime()+s);v+="</tr>"}v+="</table><br/>\n",k=new Date(Date.UTC(k.getUTCFullYear(),k.getUTCMonth(),1,b))}}return t="",o.forEach(function(e){t+=e.text}),v+e("table",t,n('class="schedule"'))+"\n\n"})}catch(e){console.log(e)}return t}function h(t,r){var n=/^.+\n:(?=[ \t])/.source,a="(s*\n|[: \t].+\n)+";return t=t.rp(RegExp("("+n+a+")+","gm"),function(t){var n=[],a=null;t.split("\n").forEach(function(e,t){0===e.trim().length?a&&(a.definition+="\n"):/\s/.test(e[0])||":"===e[0]?(":"===e[0]&&(e=" "+e.ss(1)),a.definition+=e+"\n"):(a={term:e,definition:""},n.push(a))});var i=0;n.forEach(function(e){i=/\n\s*\n/.test(e.definition.trim())?1/0:J(i,o(s(e.definition)).length)});var c="";if(i<160){var l=r("valign=top");n.forEach(function(t){c+=e("tr",e("td",e("dt",t.term))+e("td",e("dd",t.definition)),l)}),c=e("table",c)}else n.forEach(function(t){c+=e("dt",t.term)+e("dd","\n\n"+t.definition)});return e("dl",c)})}function b(t,n){var i="",o="",c=[0],l=0,u=0,d={};t=t.rp(/<h([1-6])>(.*?)<\/h\1>/gi,function(t,r,a){r=parseInt(r),a=a.trim();for(var g=l;g<r;++g)c[g]=0;c.splice(r,l-r),l=r,++c[l-1];var p=c.join("."),m="toc"+p;return d[s(a).trim().toLowerCase()]=p,r<=3&&(i+=Array(r).join("&nbsp;&nbsp;")+'<a href="#'+m+'" class="level'+r+'">'+p+"&nbsp; "+a+"</a><br/>\n",1===r?o+=' &middot; <a href="#'+m+'">'+a+"</a>":++u),e("a","",n('name="'+m+'"'))+t}),o.length>0&&(o=o.ss(10));var g=c[0],p=g+u,m=t.regexIndexOf(/((<a\s+\S+><\/a>)\s*)*?<h\d>/i);m===-1&&(m=0);var f='<div class="afterTitles"></div>',h=t.indexOf(f);h===-1?h=0:h+=f.length;var b=r("tocStyle"),x="";switch("auto"!==b&&""!==b||(b=p<4&&g<=1||t.length<2048?"none":g<7&&p/g<2.5?"short":m===-1||m/55>p?"medium":"long"),b){case"none":case"":break;case"short":x='<div class="shortTOC">'+o+"</div>";break;case"medium":x='<div class="mediumTOC"><center><b>'+a("contents")+"</b></center><p>"+i+"</p></div>";break;case"long":h=m,x='<div class="longTOC"><div class="tocHeader">Contents</div><p>'+i+"</p></div>";break;default:console.log('markdeepOptions.tocStyle = "'+b+'" specified in your document is not a legal value')}return t=t.ss(0,h)+x+t.ss(h),[t,d]}function x(e){return e.rp(/([\.\[\]\(\)\*\+\?\^\$\\\{\}\|])/g,"\\$1")}function y(e,t){return!(!e||!t)&&(e=e.match(/\n/g),t=t.match(/\n/g),e&&e.length>1&&t&&t.length>1)}function v(t,r){function o(e){var t=(T.push(e)-1).toString(A);for(t=t.rp(/x/gi,"z");t.length<S;)t="0"+t;return j+t+j}function l(e){var t=parseInt(e.ss(1,e.length-1).rp(/z/g,"x"),A);return T[t]}function u(e,t){return o(t)}function v(e,t,r){return t+o(r)}function _(t){return function(r,n){return"\n<a "+o('name="'+c(s(n))+'"')+"></a>"+e("h"+t,n)+"\n\n"}}function w(t){var r=d(t);if(r.q){var n=/^\n*[ \t]*\[[^\n]+\][ \t]*(?=\n)/;r.u=r.u.rp(n,function(t){return t=t.trim(),t=t.ss(1,t.length-1),e("center",e("div",t,o('class="imagecaption"')))});var a=M(r.q,r.s);return r.p+a+"\n"+w(r.u)}return t}var C={},N=0,k={},j="\ue010",A=35,T=[],S=4,E=RegExp(j+"[0-9a-wyz]{"+S+","+S+"}"+j,"g");void 0===r&&(r=!0),void 0!==t.innerHTML&&(t=t.innerHTML),t="\n\n"+t,t=t.rp(/<script\s+type\s*=\s*['"]preformatted['"]\s*>([\s\S]*?)<\/script>/gi,"$1");var B=function(r,n){var a=RegExp("\n"+n+"{3,}(.*)\n([\\s\\S]+?)\n"+n+"{3,}\n([ \t]*\\[.*\\])?","g");t=t.rp(a,function(t,n,a,i){var s="\n";i&&(i=i.trim(),s+="<div "+o('class="listingcaption '+r+'"')+">"+i.ss(1,i.length-1)+"</div>\n"),n=n?n.trim():n,n=n?[n]:void 0;var c=hljs.highlightAuto(a,n);return s+o(e("pre",e("code",c.value),'class="listing '+r+'"'))+"\n"})};B("tilde","~"),B("backtick","`"),t=t.rp(/(<code\b.*?<\/code>)/gi,u),t=w(t),t=t.rp(/<svg( .*?)?>([\s\S]*?)<\/svg>/gi,function(e,t,r){return"<svg"+o(t)+">"+o(r)+"</svg>"}),t=t.rp(/<style>([\s\S]*?)<\/style>/gi,function(t,r){return e("style",o(r))}),t=t.rp(/<img\s+src=(["'])[\s\S]*?\1\s*>/gi,function(e,t){return"<img "+o(e.ss(5,e.length-1))+">"}),t=t.rp(/(`)(.+?(?:\n.+?)?)`(?!\d)/g,e("code","$2")),t=t.rp(/(<code(?: .*?)?>)([\s\S]*?)<\/code>/gi,function(e,t,r){return o(t+i(r)+"</code>")}),t=t.rp(/(<pre\b[\s\S]*?<\/pre>)/gi,u),t=t.rp(/(<\w[^ \n<>]*?[ \t]+)(.*?)(?=\/?>)/g,v),t=t.rp(/(\$\$[\s\S]+?\$\$)/g,u),t=t.rp(/((?:[^\w\d]))\$(\S(?:[^\$]*?\S(?!US|Can))??)\$(?![\w\d])/g,"$1\\($2\\)"),t=t.rp(/((?:[^\w\d]))\$([ \t][^\$]+?[ \t])\$(?![\w\d])/g,"$1\\($2\\)"),t=t.rp(/(\\\([\s\S]+?\\\))/g,u),t=t.rp(/(\\begin\{equation\}[\s\S]*?\\end\{equation\})/g,u),t=t.rp(/(\\begin\{eqnarray\}[\s\S]*?\\end\{eqnarray\})/g,u),t=t.rp(/(\\begin\{equation\*\}[\s\S]*?\\end\{equation\*\})/g,u),t=t.rp(/(?:^|\n)(.+?)\n[ \t]*={3,}[ \t]*\n/g,_(1)),t=t.rp(/(?:^|\n)(.+?)\n[ \t]*-{3,}[ \t]*\n/g,_(2));for(var D=6;D>0;--D)t=t.rp(RegExp(/^[ \t]*/.source+"#{"+D+","+D+"}(?:[ \t])([^\n#]+)#*[ \t]*\n","gm"),_(D)),t=t.rp(RegExp(/^[ \t]*/.source+"\\(#{"+D+","+D+"}\\)(?:[ \t])([^\n#]+)\\(?#*\\)?\\n[ \t]*\n","gm"),e("div","$1",o('class="nonumberh'+D+'"'))+"\n\n\n");t=t.rp(/\n[ \t]*((\*|-|_)[ \t]*){3,}[ \t]*\n/g,"\n<hr/>\n");var L=o('class="fancyquote"');t=t.rp(/\n>[ \t]*"(.*(?:\n>.*)*)"[ \t]*(?:\n>[ \t]*)?(\n>[ \t]{2,}\S.*)?\n/g,function(t,r,n){return e("blockquote",e("span",r.rp(/\n>/g,"\n"),L)+(n?e("span",n.rp(/\n>/g,"\n"),o('class="author"')):""),L)}),t=t.rp(/(?:\n>.*){2,}/g,function(t){return e("blockquote",t.rp(/\n>/g,"\n"))}),t=t.rp(/\s*\[\^(\S+)\](?!:)/g,function(e,t){return t=t.toLowerCase().trim(),t in C||(++N,C[t]=N),"<sup><a "+o('href="#endnote-'+t+'"')+">"+C[t]+"</a></sup>"}),t=t.rp(/\[#(\S+)\](?!:)/g,function(e,t){return t=t.trim(),"[<a "+o('href="#citation-'+t.toLowerCase()+'"')+">"+t+"</a>]"}),t=t.rp(/\n\[#(\S+)\]: ((?:.+?\n?)*)/g,function(e,t,r){return t=t.trim(),"<div "+o('class="bib"')+">[<a "+o('name="citation-'+t.toLowerCase()+'"')+"></a><b>"+t+"</b>] "+r+"</div>"}),t=p(t,o),t=t.rp(/^\[([^\^#].*?)\]:(.*?)$/gm,function(e,t,r){return k[t.toLowerCase().trim()]={link:r.trim(),used:!1},""}),t=t.rp(/(?:<|(?!<)\b)(\S+@(\S+\.)+?\S{3,}?)(?:$|>|(?=<)|(?=\s)(?!>))/g,function(e,t){return"<a "+o('href="mailto:'+t+'"')+">"+t+"</a>"});var I=function(t,r,n){n=n||"";var a,i;return/(.mp4|.m4v|.avi|.mpg|.mov)$/i.test(r)?a="<video "+o('class="markdeep" src="'+r+'"'+n+' width="480px" controls="true"')+"/>":(i=r.match(/^https:\/\/(?:www\.)?(?:youtube\.com\/\S*?v=|youtu\.be\/)([\w\d-]+)(&.*)?$/i))?a="<iframe "+o('class="markdeep" src="https://www.youtube.com/embed/'+i[1]+'"'+n+' width="480px" height="300px" frameborder="0" allowfullscreen webkitallowfullscreen mozallowfullscreen')+"></iframe>":(i=r.match(/^https:\/\/(?:www\.)?vimeo.com\/\S*?\/([\w\d-]+)$/i))?a="<iframe "+o('class="markdeep" src="https://player.vimeo.com/video/'+i[1]+'"'+n+' width="480px" height="300px" frameborder="0" allowfullscreen webkitallowfullscreen mozallowfullscreen')+"></iframe>":(a="<img "+o('class="markdeep" src="'+r+'"'+n)+"/>",/\b(width|height)\b/i.test(n)&&(a=e("a ",a,o('href="'+r+'" target="_blank"')))),a};t=t.rp(/\(http:\/\/g.gravizo.com\/g\?((?:[^\(\)]|\([^\(\)]*\))*)\)/gi,function(e,t){return"(http://g.gravizo.com/g?"+encodeURIComponent(t)+")"}),t=t.rp(/(^|[^!])\[([^\[\]]+?)\]\(([^\)]+?)(\s+[^\)]*?)?\)/g,function(e,t,r,a,i){return i=i||"",t+"<a "+o('href="'+a+'"'+i)+">"+r+"</a>"+n(a)}),t=t.rp(/(^|[^!])\[[ \t]*?\]\(([^\)]+?)\)/g,function(e,t,r){return t+"<a "+o('href="'+r+'"')+">"+r+"</a>"});var $=o('width="100%"'),R=o('valign="top"');t=t.rp(/(?:\n(?:[ \t]*!\[[^\n]*?\]\([^\)\s]+(?:[^\)]*?)?\)){2,}[ \t]*)+\n/g,function(t){var r="";return t=t.split("\n"),t.forEach(function(t){t=t.trim(),t&&(r+=e("tr",t.rp(/[ \t]*!\[[^\n]*?\]\([^\)\s]+(?:[^\)]*?)?\)/g,function(t){return e("td","\n\n"+t+"\n\n")}),R))}),"\n"+e("table",r,$)+"\n"}),t=t.rp(/(\s*)!\[\]\(([^\)\s]+)([^\)]*?)?\)(\s*)/g,function(t,r,n,a,i){var o=I(t,n,a);return y(r,i)&&(o=e("center",o)),r+o+i});for(var U=!0;U;)U=!1,t=t.rp(/(\s*)!\[([\s\S]+?)?\]\(([^\)\s]+)([^\)]*?)?\)(\s*)/,function(t,r,a,i,s,c){U=!0;var l="",u=y(r,c);s&&!u&&(s=s.rp(/((?:max-)?width)\s*:\s*[^;'"]*/g,function(e,t){return l=e+";",t+":100%"}),s=s.rp(/((?:max-)?width)\s*=\s*('\S+?'|"\S+?")/g,function(e,t,r){return l=t+":"+r.ss(1,r.length-1)+";",'style="'+t+':100%" '}));var d=I(t,i,s);return u?(r+="<center>",c="</center>"+c):l+="float:right;margin:4px 0px 0px 25px;",r+e("div",d+e("div",a+n(i),o('class="imagecaption"')),o('class="image" style="'+l+'"'))+c});t=g(t,/\*\*/,"strong",o('class="asterisk"')),t=g(t,/__/,"strong",o('class="underscore"')),t=g(t,/\*/,"em",o('class="asterisk"')),t=g(t,/_/,"em",o('class="underscore"')),t=t.rp(/\~\~([^~].*?)\~\~/g,e("del","$1")),t=t.rp(/(^|[ \t->])(")(?=\w)/gm,"$1&ldquo;"),t=t.rp(/([A-Za-z\.,:;\?!=<])(")(?=$|\W)/gm,"$1&rdquo;"),t=t.rp(/(\s)==>(\s)/g,"$1&rarr;$2"),t=t.rp(/(\s)<==(\s)/g,"$1&larr;$2"),t=t.rp(/([^-!\:\|])---([^->\:\|])/g,"$1&mdash;$2"),t=t.rp(/([^-!\:\|])--([^->\:\|])/g,"$1&mdash;$2"),t=t.rp(/(\d+\s?)x(\s?\d+)/g,"$1&times;$2"),t=t.rp(/([\s\(\[<\|])-(\d)/g,"$1&minus;$2"),t=t.rp(/(\d) - (\d)/g,"$1 &minus; $2"),t=t.rp(/\^([-+]?\d+)\b/g,"<sup>$1</sup>"),t=f(t,o),t=h(t,o),t=m(t,o),t=t.rp(/(\d+?)[ \t-]degree(?:s?)/g,"$1&deg;"),t=t.rp(/\n[\s\n]*?\n/g,"\n\n</p><p>\n\n"),t=t.rp(/\[(.+?)\]\[(.*?)\]/g,function(e,t,r){r.trim()||(r=t),r=r.toLowerCase().trim();var n=k[r];return n?(n.used=!0,"<a "+o('href="'+n.link+'"')+">"+t+"</a>"):(console.log("Reference link '"+r+"' never defined"),"?")}),t=t.rp(/\n\[\^(\S+)\]: ((?:.+?\n?)*)/g,function(e,t,r){return t=t.toLowerCase().trim(),t in C?"\n<div "+o('class="endnote"')+"><a "+o('name="endnote-'+t+'"')+"></a><sup>"+C[t]+"</sup> "+r+"</div>":"\n"});var z=t.match(/<h([1-6])>(.*?)<\/h\1>/gi);z&&z.forEach(function(e){e=s(e.ss(4,e.length-5)).trim();var r="<a "+o('href="#'+c(e)+'"')+">";t=t.rp(RegExp("(\\b"+x(e)+")(?=\\s"+a("subsection")+"|\\s"+a("section")+")","gi"),r+"$1</a>")});var O={},F={};if(t=t.rp(RegExp(/($|>)\s*/.source+"("+a("figure")+"|"+a("table")+"|"+a("listing")+"|"+a("diagram")+")"+/\s+\[(.+?)\]:/.source,"gim"),function(t,r,n,a){var i=n.toLowerCase(),s=O[i]=(0|O[i])+1,l=i+"_"+c(a.toLowerCase().trim());return F[l]={number:s,used:!1,source:i+" ["+a+"]"},r+e("a","",o('name="'+l+'"'))+e("b",i[0].toUpperCase()+i.ss(1)+"&nbsp;"+s+":",o('style="font-style:normal;"'))}),t=t.rp(/\b(figure|fig\.|table|tbl\.|listing|lst.)\s+\[(.+?)\]/gi,function(e,t,r){var n=t.toLowerCase();switch(n){case"fig.":n="figure";break;case"tbl.":n="table";break;case"lst.":n="listing"}var a=n+"_"+c(r.toLowerCase().trim()),i=F[a];return i?(i.used=!0,"<a "+o('href="#'+a+'"')+">"+t+"&nbsp;"+i.number+"</a>"):(console.log("Reference to undefined '"+n+" ["+r+"]'"),t+" ?")}),t=t.rp(/(?:<|(?!<)\b)(\w{3,6}:\/\/.+?)(?:$|>|(?=<)|(?=\s|\u00A0)(?!<))/g,function(e,t){return"<a "+("s"!==t[0]&&"p"!==t[0]?o('href="'+t+'" class="url"'):"")+">"+t+"</a>"}),!r){var H=/^\s*(?:<\/p><p>\s*)<strong.*?>([^ \t\*].*?[^ \t\*])<\/strong>[ \t]*\n/.source,q=/([ {4,}\t][ \t]*\S.*\n)*/.source;t=t.rp(RegExp(H+q,"g"),function(t,r){r=r.trim();var a=t.ss(t.indexOf("\n",t.indexOf("</strong>")));return a=a?a.rp(/[ \t]*(\S.*?)\n/g,'<div class="subtitle"> $1 </div>\n'):"",e("title",s(r))+n(window.location.href,"center")+'<div class="title"> '+r+" </div>\n"+a+'<div class="afterTitles"></div>\n'})}if(t=t.rp(/^\s*<\/p>/,""),!r){var P=b(t,o);t=P[0];var V=P[1];t=t.rp(RegExp("\\b("+a("sec")+"\\.|"+a("section")+"|"+a("subsection")+")\\s\\[(.+?)\\]","gi"),function(e,t,r){var n=V[r.toLowerCase().trim()];return n?t+"  <a "+o('href="#toc'+n+'"')+">"+n+"</a>":t+" ?"})}for(;t.indexOf(j)+1;)t=t.rp(E,l);return Object.keys(k).forEach(function(e){k[e].used||console.log("Reference link '["+e+"]' is defined but never used")}),Object.keys(F).forEach(function(e){F[e].used||console.log("'"+F[e].source+"' is never referenced")}),'<span class="md">'+e("p",t)+"</span>"}function _(e){var t=e.split("\n"),r=0;t.forEach(function(e){r=J(r,e.length)});var n=Array(r+1).join(" "),a="";return t.forEach(function(e){a+=e+n.ss(e.length)+"\n"}),a}function w(e){var t=e.split("\n"),r=1/0;if(t.forEach(function(e){if(""!==e.trim()){var t=e.match(/^([ \t]*)/);t&&(r=K(r,t[0].length))}}),0===r)return e;var n="";return t.forEach(function(e){n+=e.ss(r)+"\n"}),n}function C(e){var t=e.charCodeAt(0);return t>=65&&t<=90||t>=97&&t<=122}function M(e,t){function r(e){return F.indexOf(e)+1}function n(e){return H.indexOf(e)!==-1}function a(e){return r(e)||"."===e}function o(e){return r(e)||"'"===e}function s(e){return n(e)||"<"===e||b(e)}function c(e){return n(e)||">"===e||b(e)}function l(e){return q.indexOf(e)+1}function u(e){return P.indexOf(e)+1}function d(e){return"-"===e||r(e)||h(e)}function g(e){return p(e)||h(e)||b(e)}function p(e){return"|"===e||r(e)}function m(e){return"/"===e||r(e)}function f(e){return"\\"===e||r(e)}function h(e){return O.indexOf(e)+1}function b(e){return z.indexOf(e)+1}function x(e){return V.indexOf(e)+1}function y(e,t){return this instanceof y?(void 0===t&&(void 0===e?e=t=0:e instanceof y?(t=e.y,e=e.x):console.error("Vec2 requires one Vec2 or (x, y) as an argument")),this.x=e,this.y=t,void Object.seal(this)):new y(e,t)}function v(e){var t=function(r,n){return void 0===n&&(r instanceof y?(n=r.y,r=r.x):console.error("grid requires either a Vec2 or (x, y)")),r>=0&&r<t.width&&n>=0&&n<t.height?e[n*(t.width+1)+r]:" "};return t._used=[],t.width=e.indexOf("\n"),t.height=e.split("\n").length,"\n"===e[e.length-1]&&--t.height,t.H=function(e,r){void 0===r&&(e instanceof y?(r=e.y,e=e.x):console.error("grid requires either a Vec2 or (x, y)")),e>=0&&e<t.width&&r>=0&&r<t.height&&(t._used[r*(t.width+1)+e]=!0)},t.I=function(e,t){return void 0===t&&(e instanceof y?(t=e.y,e=e.x):console.error("grid requires either a Vec2 or (x, y)")),this._used[t*(this.width+1)+e]===!0},t.J=function(e,r){void 0===r&&(r=e.x,e=e.x);var n=t(e,r-1),i=t(e,r),s=t(e,r+1),c=t(e+1,r-1),l=t(e-1,r-1);return p(i)?a(n)||"^"===n||p(n)||h(n)||o(s)||"v"===s||p(s)||h(s)||b(n)||b(s)||"_"===t(e,r-1)||"_"===l||"_"===c||(a(l)||a(c))&&(o(t(e-1,r+1))||o(t(e+1,r+1))):a(i)||"^"===i?p(s)||h(s)&&"."!==i:o(i)||"v"===i?p(n)||h(n)&&"'"!==i:!!b(i)&&(p(n)||p(s))},t.K=function(e,r){void 0===r&&(r=e.x,e=e.x);var a=t(e-2,r),i=t(e-1,r),o=t(e+0,r),l=t(e+1,r),u=t(e+2,r);return d(o)||d(i)&&h(o)?d(i)?d(l)||c(l)||d(a)||s(a):s(i)?d(l):d(l)&&(d(u)||c(u)):"<"===o?d(l)&&d(u):">"===o?d(i)&&d(a):!!n(o)&&(d(i)&&d(a)||d(l)&&d(u))},t.L=function(e,r){void 0===r&&(r=e.x,e=e.x);var i=t(e,r),s=t(e-1,r-1),c=t(e+1,r+1);return"\\"===i?f(c)||o(c)||b(c)||"v"===c||f(s)||a(s)||b(s)||"^"===s||"/"===t(e,r-1)||"/"===t(e,r+1)||"_"===c||"_"===s:"."===i?"\\"===c:"'"===i?"\\"===s:"^"===i?"\\"===c:"v"===i?"\\"===s:n(i)||b(i)||"|"===i?f(s)||f(c):void 0},t.M=function(e,r){void 0===r&&(r=e.x,e=e.x);var i=t(e,r),s=t(e-1,r+1),c=t(e+1,r-1);return"/"===i&&("\\"===t(e,r-1)||"\\"===t(e,r+1))||(m(i)?m(c)||a(c)||b(c)||"^"===c||"_"===c||m(s)||o(s)||b(s)||"v"===s||"_"===s:"."===i?"/"===s:"'"===i?"/"===c:"^"===i?"/"===s:"v"===i?"/"===c:!(!n(i)&&!b(i)&&"|"!==i)&&(m(s)||m(c)))},t.toString=function(){return e},Object.freeze(t)}function w(e,t,r,n,a){e instanceof y&&t instanceof y||console.error("Path constructor requires at least two Vec2s"),this.A=e,this.B=t,r&&(this.C=r,this.D=n?n:r),this.dashed=a||!1,Object.freeze(this)}function M(){this.da=[]}function N(e){return function(t,r){for(var n=0;n<this.da.length;++n)if(e.call(this.da[n],t,r))return!0;return!1}}function k(){this.fa=[]}function j(e,t){function r(t,r,n){var a,i,o=X(r.x-t.x),s=X(r.y-t.y);for(a=t.x,i=t.y;a!==r.x||i!==r.y;a+=o,i+=s)if(e(a,i)===n)return!0;return e(a,i)===n}for(var i=0;i<e.width;++i)for(var s=0;s<e.height;++s)if(e.J(i,s)){var c=y(i,s);do e.H(i,s),++s;while(e.J(i,s));var l=y(i,s-1),u=e(c),m=e(c.x,c.y-1);(!n(u)&&("-"===m||"_"===m||"_"===e(c.x-1,c.y-1)||"_"===e(c.x+1,c.y-1)||o(m))||h(m))&&(c.y-=.5);var f=e(l),x=e(l.x,l.y+1);(!n(f)&&("-"===x||a(x))||h(x)||"_"===e(l.x-1,l.y)||"_"===e(l.x+1,l.y))&&(l.y+=.5),c.x===l.x&&c.y===l.y||t.ea(new w(c,l))}else"'"===e(i,s)&&("-"===e(i-1,s)&&"_"===e(i+1,s-1)&&!g(e(i-1,s-1))||"_"===e(i-1,s-1)&&"-"===e(i+1,s)&&!g(e(i+1,s-1)))?t.ea(new w(y(i,s-.5),y(i,s))):"."===e(i,s)&&("_"===e(i-1,s)&&"-"===e(i+1,s)&&!g(e(i+1,s+1))||"-"===e(i-1,s)&&"_"===e(i+1,s)&&!g(e(i-1,s+1)))&&t.ea(new w(y(i,s),y(i,s+.5)));for(var s=0;s<e.height;++s)for(var i=0;i<e.width;++i)if(e.K(i,s)){var c=y(i,s);do e.H(i,s),++i;while(e.K(i,s));var l=y(i-1,s);!n(e(c.x-1,c.y))&&(a(e(c))&&g(e(c.x-1,c.y+1))||o(e(c))&&g(e(c.x-1,c.y-1)))&&++c.x,!n(e(l.x+1,l.y))&&(a(e(l))&&g(e(l.x+1,l.y+1))||o(e(l))&&g(e(l.x+1,l.y-1)))&&--l.x,c.x===l.x&&c.y===l.y||t.ea(new w(c,l))}for(var v=-e.height;v<e.width;++v)for(var i=v,s=0;s<e.height;++s,++i)if(e.L(i,s)){var c=y(i,s);do++i,++s;while(e.L(i,s));var l=y(i-1,s-1);if(r(c,l,"\\")){for(var _=c.x;_<=l.x;++_)e.H(_,c.y+(_-c.x));var M=e(c),u=e(c.x,c.y-1),N=e(c.x-1,c.y-1);"/"===u||"_"===N||"_"===u||!n(M)&&(d(N)||p(N))?(c.x-=.5,c.y-=.5):b(N)&&(c.x-=.25,c.y-=.25);var k=(e(l),e(l.x+1,l.y+1));"/"===e(l.x,l.y+1)||"_"===e(l.x+1,l.y)||"_"===e(l.x-1,l.y)||!n(e(l))&&(d(k)||p(k))?(l.x+=.5,l.y+=.5):b(k)&&(l.x+=.25,l.y+=.25),t.ea(new w(c,l))}}for(var v=-e.height;v<e.width;++v)for(var i=v,s=e.height-1;s>=0;--s,++i)if(e.M(i,s)){var c=y(i,s);do++i,--s;while(e.M(i,s));var l=y(i-1,s+1);if(r(c,l,"/")){for(var _=c.x;_<=l.x;++_)e.H(_,c.y-(_-c.x));var u=e(l.x,l.y-1),j=e(l.x+1,l.y-1);e(l);"\\"===u||"_"===u||"_"===j||!n(e(l))&&(d(j)||p(j))?(l.x+=.5,l.y-=.5):b(j)&&(l.x+=.25,l.y-=.25);var A=e(c.x-1,c.y+1),M=e(c);"\\"===e(c.x,c.y+1)||"_"===e(c.x-1,c.y)||"_"===e(c.x+1,c.y)||!n(e(c))&&(d(A)||p(A))?(c.x-=.5,c.y+=.5):b(A)&&(c.x-=.25,c.y+=.25),t.ea(new w(c,l))}}for(var s=0;s<e.height;++s)for(var i=0;i<e.width;++i){var T=e(i,s);a(T)&&(d(e(i-1,s))&&p(e(i+1,s+1))&&(e.H(i-1,s),e.H(i,s),e.H(i+1,s+1),t.ea(new w(y(i-1,s),y(i+1,s+1),y(i+1.1,s),y(i+1,s+1)))),d(e(i+1,s))&&p(e(i-1,s+1))&&(e.H(i-1,s+1),e.H(i,s),e.H(i+1,s),t.ea(new w(y(i+1,s),y(i-1,s+1),y(i-1.1,s),y(i-1,s+1))))),")"!==T&&!b(T)||"."!==e(i-1,s-1)||"'"!==e(i-1,s+1)||(e.H(i,s),e.H(i-1,s-1),e.H(i-1,s+1),t.ea(new w(y(i-2,s-1),y(i-2,s+1),y(i+.6,s-1),y(i+.6,s+1)))),"("!==T&&!b(T)||"."!==e(i+1,s-1)||"'"!==e(i+1,s+1)||(e.H(i,s),e.H(i+1,s-1),e.H(i+1,s+1),t.ea(new w(y(i+2,s-1),y(i+2,s+1),y(i-.6,s-1),y(i-.6,s+1)))),o(T)&&(d(e(i-1,s))&&p(e(i+1,s-1))&&(e.H(i-1,s),e.H(i,s),e.H(i+1,s-1),t.ea(new w(y(i-1,s),y(i+1,s-1),y(i+1.1,s),y(i+1,s-1)))),d(e(i+1,s))&&p(e(i-1,s-1))&&(e.H(i-1,s-1),e.H(i,s),e.H(i+1,s),t.ea(new w(y(i+1,s),y(i-1,s-1),y(i-1.1,s),y(i-1,s-1)))))}for(var s=0;s<e.height;++s)for(var i=0;i<e.width-2;++i){var S=e(i-1,s);if(!("_"!==e(i,s)||"_"!==e(i+1,s)||C(e(i+2,s))&&"_"!==S||C(S)&&"_"!==e(i+2,s))){var E=e(i-2,s),c=y(i-.5,s+.5);"|"===S||"|"===e(i-1,s+1)||"."===S||"'"===e(i-1,s+1)?(c.x-=.5,"."!==S||"-"!==E&&"."!==E||"("!==e(i-2,s+1)||(c.x-=.5)):"/"===S&&(c.x-=1),"("===S&&"("===E&&"'"===e(i,s+1)&&"."===e(i,s-1)&&(c.x+=.5),S=E=void 0;do e.H(i,s),++i;while("_"===e(i,s));var l=y(i-.5,s+.5),T=e(i,s),B=e(i+1,s),f=e(i,s+1);"|"===T||"|"===f||"."===T||"'"===f?(l.x+=.5,"."!==T||"-"!==B&&"."!==B||")"!==e(i+1,s+1)||(l.x+=.5)):"\\"===T&&(l.x+=1),")"===T&&")"===B&&"'"===e(i-1,s+1)&&"."===e(i-1,s-1)&&(l.x+=-.5),t.ea(new w(c,l))}}}function A(e,t,r){function n(e){return" "===e||/[^a-zA-Z0-9]|[ov]/.test(e)}function a(e,t,r,a){return(n(t)||b(t))&&(n(e)||b(e))&&n(a)&&n(r)}for(var i=0;i<e.width;++i)for(var o=0;o<e.height;++o){var s=e(i,o),c=o;if(h(s))t.Z(i,c-.5)&&t.U(i,c+.5)&&(r.ea(i,c,s),e.H(i,c));else if(b(s)){var d=e(i,c-1),g=e(i,c+1),p=e(i-1,c),m=e(i+1,c);(t.aa(i-1,c)||t.$(i+1,c)||t.Z(i,c-1)||t.U(i,c+1)||t.U(i,c)||t.Z(i,c)||a(d,g,p,m))&&(r.ea(i,c,s),e.H(i,c))}else if(l(s))r.ea(i,c,s),e.H(i,c);else if(u(s))r.ea(i,c,s),e.H(i,c);else{var f=0;">"===s&&(t.aa(i,c)||t.ca(i,c))?(b(e(i+1,c))&&(f=-.5),r.ea(i+f,c,">",0),e.H(i,c)):"<"===s&&(t.$(i,c)||t.ca(i,c))?(b(e(i-1,c))&&(f=.5),r.ea(i+f,c,">",180),e.H(i,c)):"^"===s?t.U(i,c-.5)?(r.ea(i,c-.5,">",270),e.H(i,c)):t.U(i,c)?(r.ea(i,c,">",270),e.H(i,c)):t.V(i+.5,c-.5)?(r.ea(i+.5,c-.5,">",270+$),e.H(i,c)):t.V(i+.25,c-.25)?(r.ea(i+.25,c-.25,">",270+$),e.H(i,c)):t.V(i,c)?(r.ea(i,c,">",270+$),e.H(i,c)):t.X(i,c)?(r.ea(i,c,s,270-$),e.H(i,c)):t.X(i-.5,c-.5)?(r.ea(i-.5,c-.5,s,270-$),e.H(i,c)):t.X(i-.25,c-.25)?(r.ea(i-.25,c-.25,s,270-$),e.H(i,c)):t.ba(i,c)&&(r.ea(i,c-.5,">",270),e.H(i,c)):"v"===s&&(t.Z(i,c+.5)?(r.ea(i,c+.5,">",90),e.H(i,c)):t.Z(i,c)?(r.ea(i,c,">",90),e.H(i,c)):t.W(i,c)?(r.ea(i,c,">",90+$),e.H(i,c)):t.W(i-.5,c+.5)?(r.ea(i-.5,c+.5,">",90+$),e.H(i,c)):t.W(i-.25,c+.25)?(r.ea(i-.25,c+.25,">",90+$),e.H(i,c)):t.Y(i,c)?(r.ea(i,c,">",90-$),e.H(i,c)):t.Y(i+.5,c+.5)?(r.ea(i+.5,c+.5,">",90-$),e.H(i,c)):t.Y(i+.25,c+.25)?(r.ea(i+.25,c+.25,">",90-$),e.H(i,c)):t.ba(i,c)&&(r.ea(i,c+.5,">",90),e.H(i,c)))}}}e=_(e);var T="\ue004";e=e.rp(/([a-z]|[A-Z])o([a-z]|[A-Z])/g,"$1"+T+"$2");var S=8,I=2,$=180*Math.atan(1/I)/Math.PI,R=1e-6,U=">v<^",z="o*",O="()",F="+",H=F+".'",q="\u2591\u2592\u2593\u2594\u2589",P="\u25e2\u25e3\u25e4\u25e5",V=U+z+O+q+P;y.prototype.toString=y.prototype.toSVG=function(){return""+this.x*S+","+this.y*S*I+" "};var W=w.prototype;W.N=function(){return this.B.x===this.A.x},W.O=function(){return this.B.y===this.A.y},W.P=function(){var e=this.B.x-this.A.x,t=this.B.y-this.A.y;return Q(t+e)<R},W.R=function(){var e=this.B.x-this.A.x,t=this.B.y-this.A.y;return Q(t-e)<R},W.S=function(){return void 0!==this.C},W.T=function(e,t){return void 0===t&&(t=e.y,e=e.x),this.A.x===e&&this.A.y===t||this.B.x===e&&this.B.y===t},W.U=function(e,t){return void 0===t&&(t=e.y,e=e.x),this.N()&&this.A.x===e&&K(this.A.y,this.B.y)===t},W.V=function(e,t){return!!this.P()&&(void 0===t&&(t=e.y,e=e.x),this.A.y<this.B.y?this.A.x===e&&this.A.y===t:this.B.x===e&&this.B.y===t)},W.W=function(e,t){return!!this.P()&&(void 0===t&&(t=e.y,e=e.x),this.B.y<this.A.y?this.A.x===e&&this.A.y===t:this.B.x===e&&this.B.y===t)},W.X=function(e,t){return!!this.R()&&(void 0===t&&(t=e.y,e=e.x),this.A.y<this.B.y?this.A.x===e&&this.A.y===t:this.B.x===e&&this.B.y===t)},W.Y=function(e,t){return!!this.R()&&(void 0===t&&(t=e.y,e=e.x),this.B.y<this.A.y?this.A.x===e&&this.A.y===t:this.B.x===e&&this.B.y===t)},W.Z=function(e,t){return void 0===t&&(t=e.y,e=e.x),this.N()&&this.A.x===e&&J(this.A.y,this.B.y)===t},W.$=function(e,t){return void 0===t&&(t=e.y,e=e.x),this.O()&&this.A.y===t&&K(this.A.x,this.B.x)===e},W.aa=function(e,t){return void 0===t&&(t=e.y,e=e.x),this.O()&&this.A.y===t&&J(this.A.x,this.B.x)===e},W.ba=function(e,t){return void 0===t&&(t=e.y,e=e.x),this.N()&&this.A.x===e&&K(this.A.y,this.B.y)<=t&&J(this.A.y,this.B.y)>=t},W.ca=function(e,t){return void 0===t&&(t=e.y,e=e.x),this.O()&&this.A.y===t&&K(this.A.x,this.B.x)<=e&&J(this.A.x,this.B.x)>=e},W.toSVG=function(){var e='<path d="M '+this.A;return e+=this.S()?"C "+this.C+this.D+this.B:"L "+this.B,e+='" style="fill:none;"',this.dashed&&(e+=' stroke-dasharray="3,6"'),e+="/>"};var Z=M.prototype;Z.ea=function(e){this.da.push(e)},Z.U=N(W.U),Z.V=N(W.V),Z.X=N(W.X),Z.W=N(W.W),Z.Y=N(W.Y),Z.Z=N(W.Z),Z.$=N(W.$),Z.aa=N(W.aa),Z.T=N(W.T),Z.ba=N(W.ba),Z.ca=N(W.ca),Z.toSVG=function(){for(var e="",t=0;t<this.da.length;++t)e+=this.da[t].toSVG()+"\n";return e};var G=k.prototype;G.ea=function(e,t,r,n){void 0===r&&(r=t,t=e.y,e=e.x),x(r)||console.error("Illegal decoration character: "+r);var a={C:y(e,t),type:r,angle:n||0};b(r)?this.fa.push(a):this.fa.unshift(a)},G.toSVG=function(){for(var e="",t=0;t<this.fa.length;++t){var r=this.fa[t],n=r.C;if(h(r.type)){var a=")"===r.type?.75:-.75,i=y(n.x,n.y-.5),o=y(n.x,n.y+.5),s=y(n.x+a,n.y-.5),c=y(n.x+a,n.y+.5);e+='<path d="M '+o+" C "+c+s+i+'" style="fill:none;"/>'}else if(b(r.type))e+='<circle cx="'+n.x*S+'" cy="'+n.y*S*I+'" r="'+(S-L)+'" class="'+("*"===r.type?"closed":"open")+'dot"/>';else if(l(r.type)){var d=Math.round(63.75*(3-q.indexOf(r.type)));e+='<rect x="'+(n.x-.5)*S+'" y="'+(n.y-.5)*S*I+'" width="'+S+'" height="'+S*I+'" fill="rgb('+d+","+d+","+d+')"/>'}else if(u(r.type)){var g=P.indexOf(r.type),p=.5-(1&g),m=.5-(g>>1);p*=X(m);var f=y(n.x+p,n.y-m),i=y(n.x+p,n.y+m),o=y(n.x-p,n.y+m);e+='<polygon points="'+f+i+o+'" style="stroke:none"/>\n'}else{var f=y(n.x+1,n.y),i=y(n.x-.5,n.y-.35),o=y(n.x-.5,n.y+.35);e+='<polygon points="'+f+i+o+'"  style="stroke:none" transform="rotate('+r.angle+","+n+')"/>\n'}}return e};var Y=v(e),ee=new M,te=new k;j(Y,ee),A(Y,ee,te);var re='<svg class="diagram" xmlns="http://www.w3.org/2000/svg" version="1.1" height="'+(Y.height+1)*S*I+'" width="'+(Y.width+1)*S+'"';if("floatleft"===t?re+=' style="float:left;margin:15px 30px 15px 0;"':"floatright"===t?re+=' style="float:right;margin:15px 0 15px 30px;"':"center"===t&&(re+=' style="margin:0 auto 0 auto;"'),re+='><g transform="translate('+y(1,1)+')">\n',E){re+='<g style="opacity:0.1">\n';for(var ne=0;ne<Y.width;++ne)for(var ae=0;ae<Y.height;++ae)re+='<rect x="'+((ne-.5)*S+1)+'" + y="'+((ae-.5)*S*I+2)+'" width="'+(S-2)+'" height="'+(S*I-2)+'" style="fill:',
re+=Y.I(ne,ae)?"red;":" "===Y(ne,ae)?"gray;opacity:0.05":"blue;",re+='"/>\n';re+="</g>\n"}if(re+=ee.toSVG(),re+=te.toSVG(),!D){re+='<g transform="translate(0,0)">';for(var ae=0;ae<Y.height;++ae)for(var ne=0;ne<Y.width;++ne){var ie=Y(ne,ae);/[\u2B22\u2B21]/.test(ie)?re+='<text text-anchor="middle" x="'+ne*S+'" y="'+(4+ae*S*I)+'" style="font-size:20.5px">'+i(ie)+"</text>":" "===ie||Y.I(ne,ae)||(re+='<text text-anchor="middle" x="'+ne*S+'" y="'+(4+ae*S*I)+'">'+i(ie)+"</text>")}re+="</g>"}if(B){re+='<g transform="translate(2,2)">\n';for(var ne=0;ne<Y.width;++ne)for(var ae=0;ae<Y.height;++ae){var ie=Y(ne,ae);" "!==ie&&(re+='<text text-anchor="middle" x="'+ne*S+'" y="'+(4+ae*S*I)+'" style="fill:#F00;font-family:Menlo,monospace;font-size:12px;text-align:center">'+i(ie)+"</text>")}re+="</g>"}return re+="</g></svg>",re=re.rp(RegExp(T,"g"),"o")}function N(e){return e.search(/markdeep\S*?\.js$/i)!==-1}function k(e){return Array.prototype.slice.call(e)}function j(){parent.postMessage(ce+"="+re,"*")}function A(e){return e&&e.ss(0,e.lastIndexOf("/")+1)}var T='<div class="markdeepFooter"><i>formatted by <a href="http://casual-effects.com/markdeep" style="color:#999">Markdeep&nbsp;0.16&nbsp;&nbsp;</a></i><div style="display:inline-block;font-size:13px;font-family:\'Times New Roman\',serif;vertical-align:middle;transform:translate(-3px,-1px)rotate(135deg);">&#x2712;</div></div>',S=String.prototype;S.rp=S.replace,S.ss=S.substring,S.regexIndexOf=function(e,t){var r=this.ss(t||0).search(e);return r>=0?r+(t||0):r};var E=!1,B=E,D=B,L=2,I="*",$=Array(6).join(I);!function(e){var t="object"==typeof window&&window||"object"==typeof self&&self;"undefined"!=typeof exports?e(exports):t&&(t.hljs=e({}),"function"==typeof define&&define.amd&&define([],function(){return t.hljs}))}(function(e){function t(e){return e.replace(/[&<>]/gm,function(e){return S[e]})}function r(e){return e.nodeName.toLowerCase()}function n(e,t){var r=e&&e.exec(t);return r&&0===r.index}function a(e){return N.test(e)}function i(e){var t,r,n,i,o=e.className+" ";if(o+=e.parentNode?e.parentNode.className:"",r=k.exec(o))return v(r[1])?r[1]:"no-highlight";for(o=o.split(/\s+/),t=0,n=o.length;n>t;t++)if(i=o[t],a(i)||v(i))return i}function o(e,t){var r,n={};for(r in e)n[r]=e[r];if(t)for(r in t)n[r]=t[r];return n}function s(e){var t=[];return function e(n,a){for(var i=n.firstChild;i;i=i.nextSibling)3===i.nodeType?a+=i.nodeValue.length:1===i.nodeType&&(t.push({event:"start",offset:a,node:i}),a=e(i,a),r(i).match(/br|hr|img|input/)||t.push({event:"stop",offset:a,node:i}));return a}(e,0),t}function c(e,n,a){function i(){return e.length&&n.length?e[0].offset!==n[0].offset?e[0].offset<n[0].offset?e:n:"start"===n[0].event?e:n:e.length?e:n}function o(e){function n(e){return" "+e.nodeName+'="'+t(e.value)+'"'}u+="<"+r(e)+_.map.call(e.attributes,n).join("")+">"}function s(e){u+="</"+r(e)+">"}function c(e){("start"===e.event?o:s)(e.node)}for(var l=0,u="",d=[];e.length||n.length;){var g=i();if(u+=t(a.substr(l,g[0].offset-l)),l=g[0].offset,g===e){d.reverse().forEach(s);do c(g.splice(0,1)[0]),g=i();while(g===e&&g.length&&g[0].offset===l);d.reverse().forEach(o)}else"start"===g[0].event?d.push(g[0].node):d.pop(),c(g.splice(0,1)[0])}return u+t(a.substr(l))}function l(e){function t(e){return e&&e.source||e}function r(r,n){return RegExp(t(r),"m"+(e.cI?"i":"")+(n?"g":""))}function n(a,i){if(!a.compiled){if(a.compiled=!0,a.k=a.k||a.bK){var s={},c=function(t,r){e.cI&&(r=r.toLowerCase()),r.split(" ").forEach(function(e){var r=e.split("|");s[r[0]]=[t,r[1]?+r[1]:1]})};"string"==typeof a.k?c("keyword",a.k):w(a.k).forEach(function(e){c(e,a.k[e])}),a.k=s}a.lR=r(a.l||/\w+/,!0),i&&(a.bK&&(a.b="\\b("+a.bK.split(" ").join("|")+")\\b"),a.b||(a.b=/\B|\b/),a.bR=r(a.b),a.e||a.eW||(a.e=/\B|\b/),a.e&&(a.eR=r(a.e)),a.tE=t(a.e)||"",a.eW&&i.tE&&(a.tE+=(a.e?"|":"")+i.tE)),a.i&&(a.iR=r(a.i)),null==a.r&&(a.r=1),a.c||(a.c=[]);var l=[];a.c.forEach(function(e){e.v?e.v.forEach(function(t){l.push(o(e,t))}):l.push("self"===e?a:e)}),a.c=l,a.c.forEach(function(e){n(e,a)}),a.starts&&n(a.starts,i);var u=a.c.map(function(e){return e.bK?"\\.?("+e.b+")\\.?":e.b}).concat([a.tE,a.i]).map(t).filter(Boolean);a.t=u.length?r(u.join("|"),!0):{exec:function(){return null}}}}n(e)}function u(e,r,a,i){function o(e,t){for(var r=0;r<t.c.length;r++)if(n(t.c[r].bR,e))return t.c[r]}function s(e,t){if(n(e.eR,t)){for(;e.endsParent&&e.parent;)e=e.parent;return e}return e.eW?s(e.parent,t):void 0}function c(e,t){return!a&&n(t.iR,e)}function g(e,t){var r=y.cI?t[0].toLowerCase():t[0];return e.k.hasOwnProperty(r)&&e.k[r]}function p(e,t,r,n){var a=n?"":T.classPrefix,i='<span class="'+a,o=r?"":A;return i+=e+'">',i+t+o}function m(){var e,r,n,a;if(!w.k)return t(k);for(a="",r=0,w.lR.lastIndex=0,n=w.lR.exec(k);n;)a+=t(k.substr(r,n.index-r)),e=g(w,n),e?(j+=e[1],a+=p(e[0],t(n[0]))):a+=t(n[0]),r=w.lR.lastIndex,n=w.lR.exec(k);return a+t(k.substr(r))}function f(){var e="string"==typeof w.sL;if(e&&!C[w.sL])return t(k);var r=e?u(w.sL,k,!0,M[w.sL]):d(k,w.sL.length?w.sL:void 0);return w.r>0&&(j+=r.r),e&&(M[w.sL]=r.top),p(r.language,r.value,!1,!0)}function h(){N+=null!=w.sL?f():m(),k=""}function b(e){N+=e.cN?p(e.cN,"",!0):"",w=Object.create(e,{parent:{value:w}})}function x(e,t){if(k+=e,null==t)return h(),0;var r=o(t,w);if(r)return r.skip?k+=t:(r.eB&&(k+=t),h(),r.rB||r.eB||(k=t)),b(r,t),r.rB?0:t.length;var n=s(w,t);if(n){var a=w;a.skip?k+=t:(a.rE||a.eE||(k+=t),h(),a.eE&&(k=t));do w.cN&&(N+=A),w.skip||(j+=w.r),w=w.parent;while(w!==n.parent);return n.starts&&b(n.starts,""),a.rE?0:t.length}if(c(t,w))throw Error('Illegal lexeme "'+t+'" for mode "'+(w.cN||"<unnamed>")+'"');return k+=t,t.length||1}var y=v(e);if(!y)throw Error('Unknown language: "'+e+'"');l(y);var _,w=i||y,M={},N="";for(_=w;_!==y;_=_.parent)_.cN&&(N=p(_.cN,"",!0)+N);var k="",j=0;try{for(var S,E,B=0;w.t.lastIndex=B,S=w.t.exec(r),S;)E=x(r.substr(B,S.index-B),S[0]),B=S.index+E;for(x(r.substr(B)),_=w;_.parent;_=_.parent)_.cN&&(N+=A);return{r:j,value:N,language:e,top:w}}catch(e){if(e.message&&-1!==e.message.indexOf("Illegal"))return{r:0,value:t(r)};throw e}}function d(e,r){r=r||T.languages||w(C);var n={r:0,value:t(e)},a=n;return r.filter(v).forEach(function(t){var r=u(t,e,!1);r.language=t,r.r>a.r&&(a=r),r.r>n.r&&(a=n,n=r)}),a.language&&(n.second_best=a),n}function g(e){return T.tabReplace||T.useBR?e.replace(j,function(e,t){return T.useBR&&"\n"===e?"<br>":T.tabReplace?t.replace(/\t/g,T.tabReplace):void 0}):e}function p(e,t,r){var n=t?M[t]:r,a=[e.trim()];return e.match(/\bhljs\b/)||a.push("hljs"),-1===e.indexOf(n)&&a.push(n),a.join(" ").trim()}function m(e){var t,r,n,o,l,m=i(e);a(m)||(T.useBR?(t=document.createElementNS("http://www.w3.org/1999/xhtml","div"),t.innerHTML=e.innerHTML.replace(/\n/g,"").replace(/<br[ \/]*>/g,"\n")):t=e,l=t.textContent,n=m?u(m,l,!0):d(l),r=s(t),r.length&&(o=document.createElementNS("http://www.w3.org/1999/xhtml","div"),o.innerHTML=n.value,n.value=c(r,s(o),l)),n.value=g(n.value),e.innerHTML=n.value,e.className=p(e.className,m,n.language),e.result={language:n.language,re:n.r},n.second_best&&(e.second_best={language:n.second_best.language,re:n.second_best.r}))}function f(e){T=o(T,e)}function h(){if(!h.called){h.called=!0;var e=document.querySelectorAll("pre code");_.forEach.call(e,m)}}function b(){addEventListener("DOMContentLoaded",h,!1),addEventListener("load",h,!1)}function x(t,r){var n=C[t]=r(e);n.aliases&&n.aliases.forEach(function(e){M[e]=t})}function y(){return w(C)}function v(e){return e=(e||"").toLowerCase(),C[e]||C[M[e]]}var _=[],w=Object.keys,C={},M={},N=/^(no-?highlight|plain|text)$/i,k=/\blang(?:uage)?-([\w-]+)\b/i,j=/((^(<[^>]+>|\t|)+|(?:\n)))/gm,A="</span>",T={classPrefix:"hljs-",tabReplace:null,useBR:!1,languages:void 0},S={"&":"&amp;","<":"&lt;",">":"&gt;"};return e.highlight=u,e.highlightAuto=d,e.fixMarkup=g,e.highlightBlock=m,e.configure=f,e.initHighlighting=h,e.initHighlightingOnLoad=b,e.g=x,e.h=y,e.j=v,e.inherit=o,e.IR="[a-zA-Z]\\w*",e.UIR="[a-zA-Z_]\\w*",e.NR="\\b\\d+(\\.\\d+)?",e.CNR="(-?)(\\b0[xX][a-fA-F0-9]+|(\\b\\d+(\\.\\d*)?|\\.\\d+)([eE][-+]?\\d+)?)",e.BNR="\\b(0b[01]+)",e.RSR="!|!=|!==|%|%=|&|&&|&=|\\*|\\*=|\\+|\\+=|,|-|-=|/=|/|:|;|<<|<<=|<=|<|===|==|=|>>>=|>>=|>=|>>>|>>|>|\\?|\\[|\\{|\\(|\\^|\\^=|\\||\\|=|\\|\\||~",e.BE={b:"\\\\[\\s\\S]",r:0},e.ASM={cN:"string",b:"'",e:"'",i:"\\n",c:[e.BE]},e.QSM={cN:"string",b:'"',e:'"',i:"\\n",c:[e.BE]},e.PWM={b:/\b(a|an|the|are|I'm|isn't|don't|doesn't|won't|but|just|should|pretty|simply|enough|gonna|going|wtf|so|such|will|you|your|like)\b/},e.C=function(t,r,n){var a=e.inherit({cN:"comment",b:t,e:r,c:[]},n||{});return a.c.push(e.PWM),a.c.push({cN:"doctag",b:"(?:TODO|FIXME|NOTE|BUG|XXX):",r:0}),a},e.CLCM=e.C("//","$"),e.CBCM=e.C("/\\*","\\*/"),e.HCM=e.C("#","$"),e.NM={cN:"number",b:e.NR,r:0},e.CNM={cN:"number",b:e.CNR,r:0},e.BNM={cN:"number",b:e.BNR,r:0},e.CSSNM={cN:"number",b:e.NR+"(%|em|ex|ch|rem|vw|vh|vmin|vmax|cm|mm|in|pt|pc|px|deg|grad|rad|turn|s|ms|Hz|kHz|dpi|dpcm|dppx)?",r:0},e.RM={cN:"regexp",b:/\//,e:/\/[gimuy]*/,i:/\n/,c:[e.BE,{b:/\[/,e:/\]/,r:0,c:[e.BE]}]},e.TM={cN:"title",b:e.IR,r:0},e.UTM={cN:"title",b:e.UIR,r:0},e.METHOD_GUARD={b:"\\.\\s*"+e.UIR,r:0},e}),hljs.g("lisp",function(e){var t="[a-zA-Z_\\-\\+\\*\\/\\<\\=\\>\\&\\#][a-zA-Z0-9_\\-\\+\\*\\/\\<\\=\\>\\&\\#!]*",r="\\|[^]*?\\|",n="(\\-|\\+)?\\d+(\\.\\d+|\\/\\d+)?((d|e|f|l|s|D|E|F|L|S)(\\+|\\-)?\\d+)?",a={cN:"meta",b:"^#!",e:"$"},i={cN:"literal",b:"\\b(t{1}|nil)\\b"},o={cN:"number",v:[{b:n,r:0},{b:"#(b|B)[0-1]+(/[0-1]+)?"},{b:"#(o|O)[0-7]+(/[0-7]+)?"},{b:"#(x|X)[0-9a-fA-F]+(/[0-9a-fA-F]+)?"},{b:"#(c|C)\\("+n+" +"+n,e:"\\)"}]},s=e.inherit(e.QSM,{i:null}),c=e.C(";","$",{r:0}),l={b:"\\*",e:"\\*"},u={cN:"symbol",b:"[:&]"+t},d={b:t,r:0},g={b:r},p={b:"\\(",e:"\\)",c:["self",i,s,o,d]},m={c:[o,s,l,u,p,d],v:[{b:"['`]\\(",e:"\\)"},{b:"\\(quote ",e:"\\)",k:{name:"quote"}},{b:"'"+r}]},f={v:[{b:"'"+t},{b:"#'"+t+"(::"+t+")*"}]},h={b:"\\(\\s*",e:"\\)"},b={eW:!0,r:0};return h.c=[{cN:"name",v:[{b:t},{b:r}]},b],b.c=[m,f,h,i,o,s,c,l,u,g,d],{i:/\S/,c:[o,a,i,s,c,m,f,h,d]}}),hljs.g("bash",function(e){var t={cN:"variable",v:[{b:/\$[\w\d#@][\w\d_]*/},{b:/\$\{(.*?)}/}]},r={cN:"string",b:/"/,e:/"/,c:[e.BE,t,{cN:"variable",b:/\$\(/,e:/\)/,c:[e.BE]}]},n={cN:"string",b:/'/,e:/'/};return{aliases:["sh","zsh"],l:/-?[a-z\.]+/,k:{keyword:"if then else elif fi for while in do done case esac function",literal:"true false",built_in:"break cd continue eval exec exit export getopts hash pwd readonly return shift test times trap umask unset alias bind builtin caller command declare echo enable help let local logout mapfile printf read readarray source type typeset ulimit unalias set shopt autoload bg bindkey bye cap chdir clone comparguments compcall compctl compdescribe compfiles compgroups compquote comptags comptry compvalues dirs disable disown echotc echoti emulate fc fg float functions getcap getln history integer jobs kill limit log noglob popd print pushd pushln rehash sched setcap setopt stat suspend ttyctl unfunction unhash unlimit unsetopt vared wait whence where which zcompile zformat zftp zle zmodload zparseopts zprof zpty zregexparse zsocket zstyle ztcp",_:"-ne -eq -lt -gt -f -d -e -s -l -a"},c:[{cN:"meta",b:/^#![^\n]+sh\s*$/,r:10},{cN:"function",b:/\w[\w\d_]*\s*\(\s*\)\s*\{/,rB:!0,c:[e.inherit(e.TM,{b:/\w[\w\d_]*/})],r:0},e.HCM,r,n,t]}}),hljs.g("markdown",function(e){return{aliases:["md","mkdown","mkd"],c:[{cN:"section",v:[{b:"^#{1,6}",e:"$"},{b:"^.+?\\n[=-]{2,}$"}]},{b:"<",e:">",sL:"xml",r:0},{cN:"bullet",b:"^([*+-]|(\\d+\\.))\\s+"},{cN:"strong",b:"[*_]{2}.+?[*_]{2}"},{cN:"emphasis",v:[{b:"\\*.+?\\*"},{b:"_.+?_",r:0}]},{cN:"quote",b:"^>\\s+",e:"$"},{cN:"code",v:[{b:"^```w*s*$",e:"^```s*$"},{b:"`.+?`"},{b:"^( {4}|\t)",e:"$",r:0}]},{b:"^[-\\*]{3,}",e:"$"},{b:"\\[.+?\\][\\(\\[].*?[\\)\\]]",rB:!0,c:[{cN:"string",b:"\\[",e:"\\]",eB:!0,rE:!0,r:0},{cN:"link",b:"\\]\\(",e:"\\)",eB:!0,eE:!0},{cN:"symbol",b:"\\]\\[",e:"\\]",eB:!0,eE:!0}],r:10},{b:/^\[[^\n]+\]:/,rB:!0,c:[{cN:"symbol",b:/\[/,e:/\]/,eB:!0,eE:!0},{cN:"link",b:/:\s*/,e:/$/,eB:!0}]}]}}),hljs.g("glsl",function(e){return{k:{keyword:"break continue discard do else for if return whileattribute binding buffer ccw centroid centroid varying coherent column_major const cw depth_any depth_greater depth_less depth_unchanged early_fragment_tests equal_spacing flat fractional_even_spacing fractional_odd_spacing highp in index inout invariant invocations isolines layout line_strip lines lines_adjacency local_size_x local_size_y local_size_z location lowp max_vertices mediump noperspective offset origin_upper_left out packed patch pixel_center_integer point_mode points precise precision quads r11f_g11f_b10f r16 r16_snorm r16f r16i r16ui r32f r32i r32ui r8 r8_snorm r8i r8ui readonly restrict rg16 rg16_snorm rg16f rg16i rg16ui rg32f rg32i rg32ui rg8 rg8_snorm rg8i rg8ui rgb10_a2 rgb10_a2ui rgba16 rgba16_snorm rgba16f rgba16i rgba16ui rgba32f rgba32i rgba32ui rgba8 rgba8_snorm rgba8i rgba8ui row_major sample shared smooth std140 std430 stream triangle_strip triangles triangles_adjacency uniform varying vertices volatile writeonly",type:"atomic_uint bool bvec2 bvec3 bvec4 dmat2 dmat2x2 dmat2x3 dmat2x4 dmat3 dmat3x2 dmat3x3 dmat3x4 dmat4 dmat4x2 dmat4x3 dmat4x4 double dvec2 dvec3 dvec4 float iimage1D iimage1DArray iimage2D iimage2DArray iimage2DMS iimage2DMSArray iimage2DRect iimage3D iimageBufferiimageCube iimageCubeArray image1D image1DArray image2D image2DArray image2DMS image2DMSArray image2DRect image3D imageBuffer imageCube imageCubeArray int isampler1D isampler1DArray isampler2D isampler2DArray isampler2DMS isampler2DMSArray isampler2DRect isampler3D isamplerBuffer isamplerCube isamplerCubeArray ivec2 ivec3 ivec4 mat2 mat2x2 mat2x3 mat2x4 mat3 mat3x2 mat3x3 mat3x4 mat4 mat4x2 mat4x3 mat4x4 sampler1D sampler1DArray sampler1DArrayShadow sampler1DShadow sampler2D sampler2DArray sampler2DArrayShadow sampler2DMS sampler2DMSArray sampler2DRect sampler2DRectShadow sampler2DShadow sampler3D samplerBuffer samplerCube samplerCubeArray samplerCubeArrayShadow samplerCubeShadow image1D uimage1DArray uimage2D uimage2DArray uimage2DMS uimage2DMSArray uimage2DRect uimage3D uimageBuffer uimageCube uimageCubeArray uint usampler1D usampler1DArray usampler2D usampler2DArray usampler2DMS usampler2DMSArray usampler2DRect usampler3D samplerBuffer usamplerCube usamplerCubeArray uvec2 uvec3 uvec4 vec2 vec3 vec4 void",built_in:"gl_MaxAtomicCounterBindings gl_MaxAtomicCounterBufferSize gl_MaxClipDistances gl_MaxClipPlanes gl_MaxCombinedAtomicCounterBuffers gl_MaxCombinedAtomicCounters gl_MaxCombinedImageUniforms gl_MaxCombinedImageUnitsAndFragmentOutputs gl_MaxCombinedTextureImageUnits gl_MaxComputeAtomicCounterBuffers gl_MaxComputeAtomicCounters gl_MaxComputeImageUniforms gl_MaxComputeTextureImageUnits gl_MaxComputeUniformComponents gl_MaxComputeWorkGroupCount gl_MaxComputeWorkGroupSize gl_MaxDrawBuffers gl_MaxFragmentAtomicCounterBuffers gl_MaxFragmentAtomicCounters gl_MaxFragmentImageUniforms gl_MaxFragmentInputComponents gl_MaxFragmentInputVectors gl_MaxFragmentUniformComponents gl_MaxFragmentUniformVectors gl_MaxGeometryAtomicCounterBuffers gl_MaxGeometryAtomicCounters gl_MaxGeometryImageUniforms gl_MaxGeometryInputComponents gl_MaxGeometryOutputComponents gl_MaxGeometryOutputVertices gl_MaxGeometryTextureImageUnits gl_MaxGeometryTotalOutputComponents gl_MaxGeometryUniformComponents gl_MaxGeometryVaryingComponents gl_MaxImageSamples gl_MaxImageUnits gl_MaxLights gl_MaxPatchVertices gl_MaxProgramTexelOffset gl_MaxTessControlAtomicCounterBuffers gl_MaxTessControlAtomicCounters gl_MaxTessControlImageUniforms gl_MaxTessControlInputComponents gl_MaxTessControlOutputComponents gl_MaxTessControlTextureImageUnits gl_MaxTessControlTotalOutputComponents gl_MaxTessControlUniformComponents gl_MaxTessEvaluationAtomicCounterBuffers gl_MaxTessEvaluationAtomicCounters gl_MaxTessEvaluationImageUniforms gl_MaxTessEvaluationInputComponents gl_MaxTessEvaluationOutputComponents gl_MaxTessEvaluationTextureImageUnits gl_MaxTessEvaluationUniformComponents gl_MaxTessGenLevel gl_MaxTessPatchComponents gl_MaxTextureCoords gl_MaxTextureImageUnits gl_MaxTextureUnits gl_MaxVaryingComponents gl_MaxVaryingFloats gl_MaxVaryingVectors gl_MaxVertexAtomicCounterBuffers gl_MaxVertexAtomicCounters gl_MaxVertexAttribs gl_MaxVertexImageUniforms gl_MaxVertexOutputComponents gl_MaxVertexOutputVectors gl_MaxVertexTextureImageUnits gl_MaxVertexUniformComponents gl_MaxVertexUniformVectors gl_MaxViewports gl_MinProgramTexelOffset gl_BackColor gl_BackLightModelProduct gl_BackLightProduct gl_BackMaterial gl_BackSecondaryColor gl_ClipDistance gl_ClipPlane gl_ClipVertex gl_Color gl_DepthRange gl_EyePlaneQ gl_EyePlaneR gl_EyePlaneS gl_EyePlaneT gl_Fog gl_FogCoord gl_FogFragCoord gl_FragColor gl_FragCoord gl_FragData gl_FragDepth gl_FrontColor gl_FrontFacing gl_FrontLightModelProduct gl_FrontLightProduct gl_FrontMaterial gl_FrontSecondaryColor gl_GlobalInvocationID gl_InstanceID gl_InvocationID gl_Layer gl_LightModel gl_LightSource gl_LocalInvocationID gl_LocalInvocationIndex gl_ModelViewMatrix gl_ModelViewMatrixInverse gl_ModelViewMatrixInverseTranspose gl_ModelViewMatrixTranspose gl_ModelViewProjectionMatrix gl_ModelViewProjectionMatrixInverse gl_ModelViewProjectionMatrixInverseTranspose gl_ModelViewProjectionMatrixTranspose gl_MultiTexCoord0 gl_MultiTexCoord1 gl_MultiTexCoord2 gl_MultiTexCoord3 gl_MultiTexCoord4 gl_MultiTexCoord5 gl_MultiTexCoord6 gl_MultiTexCoord7 gl_Normal gl_NormalMatrix gl_NormalScale gl_NumSamples gl_NumWorkGroups gl_ObjectPlaneQ gl_ObjectPlaneR gl_ObjectPlaneS gl_ObjectPlaneT gl_PatchVerticesIn gl_Point gl_PointCoord gl_PointSize gl_Position gl_PrimitiveID gl_PrimitiveIDIn gl_ProjectionMatrix gl_ProjectionMatrixInverse gl_ProjectionMatrixInverseTranspose gl_ProjectionMatrixTranspose gl_SampleID gl_SampleMask gl_SampleMaskIn gl_SamplePosition gl_SecondaryColor gl_TessCoord gl_TessLevelInner gl_TessLevelOuter gl_TexCoord gl_TextureEnvColor gl_TextureMatrix gl_TextureMatrixInverse gl_TextureMatrixInverseTranspose gl_TextureMatrixTranspose gl_Vertex gl_VertexID gl_ViewportIndex gl_WorkGroupID gl_WorkGroupSize gl_in gl_out EmitStreamVertex EmitVertex EndPrimitive EndStreamPrimitive abs acos acosh all any asin asinh atan atanh atomicAdd atomicAnd atomicCompSwap atomicCounter atomicCounterDecrement atomicCounterIncrement atomicExchange atomicMax atomicMin atomicOr atomicXor barrier bitCount bitfieldExtract bitfieldInsert bitfieldReverse ceil clamp cos cosh cross dFdx dFdy degrees determinant distance dot equal exp exp2 faceforward findLSB findMSB floatBitsToInt floatBitsToUint floor fma fract frexp ftransform fwidth greaterThan greaterThanEqual groupMemoryBarrier imageAtomicAdd imageAtomicAnd imageAtomicCompSwap imageAtomicExchange imageAtomicMax imageAtomicMin imageAtomicOr imageAtomicXor imageLoad imageSize imageStore imulExtended intBitsToFloat interpolateAtCentroid interpolateAtOffset interpolateAtSample inverse inversesqrt isinf isnan ldexp length lessThan lessThanEqual log log2 matrixCompMult max memoryBarrier memoryBarrierAtomicCounter memoryBarrierBuffer memoryBarrierImage memoryBarrierShared min mix mod modf noise1 noise2 noise3 noise4 normalize not notEqual outerProduct packDouble2x32 packHalf2x16 packSnorm2x16 packSnorm4x8 packUnorm2x16 packUnorm4x8 pow radians reflect refract round roundEven shadow1D shadow1DLod shadow1DProj shadow1DProjLod shadow2D shadow2DLod shadow2DProj shadow2DProjLod sign sin sinh smoothstep sqrt step tan tanh texelFetch texelFetchOffset texture texture1D texture1DLod texture1DProj texture1DProjLod texture2D texture2DLod texture2DProj texture2DProjLod texture3D texture3DLod texture3DProj texture3DProjLod textureCube textureCubeLod textureGather textureGatherOffset textureGatherOffsets textureGrad textureGradOffset textureLod textureLodOffset textureOffset textureProj textureProjGrad textureProjGradOffset textureProjLod textureProjLodOffset textureProjOffset textureQueryLevels textureQueryLod textureSize transpose trunc uaddCarry uintBitsToFloat umulExtended unpackDouble2x32 unpackHalf2x16 unpackSnorm2x16 unpackSnorm4x8 unpackUnorm2x16 unpackUnorm4x8 usubBorrow",literal:"true false"},i:'"',c:[e.CLCM,e.CBCM,e.CNM,{cN:"meta",b:"#",e:"$"}]}}),hljs.g("cpp",function(e){var t={cN:"keyword",b:"\\b[a-z\\d_]*_t\\b"},r={cN:"string",v:[{b:'(u8?|U)?L?"',e:'"',i:"\\n",c:[e.BE]},{b:'(u8?|U)?R"',e:'"',c:[e.BE]},{b:"'\\\\?.",e:"'",i:"."}]},n={cN:"number",v:[{b:"\\b(0b[01'_]+)"},{b:"\\b([\\d'_]+(\\.[\\d'_]*)?|\\.[\\d'_]+)(u|U|l|L|ul|UL|f|F|b|B)"},{b:"(-?)(\\b0[xX][a-fA-F0-9'_]+|(\\b[\\d'_]+(\\.[\\d'_]*)?|\\.[\\d'_]+)([eE][-+]?[\\d'_]+)?)"}],r:0},a={cN:"meta",b:/#\s*[a-z]+\b/,e:/$/,k:{"meta-keyword":"if else elif endif define undef warning error line pragma ifdef ifndef include"},c:[{b:/\\\n/,r:0},e.inherit(r,{cN:"meta-string"}),{cN:"meta-string",b:"<",e:">",i:"\\n"},e.CLCM,e.CBCM]},i=e.IR+"\\s*\\(",o={keyword:"int float while private char catch export virtual operator sizeof dynamic_cast|10 typedef const_cast|10 const struct for static_cast|10 union namespace unsigned long volatile static protected bool template mutable if public friend do goto auto void enum else break extern using class asm case typeid short reinterpret_cast|10 default double register explicit signed typename try this switch continue inline delete alignof constexpr decltype noexcept static_assert thread_local restrict _Bool complex _Complex _Imaginary atomic_bool atomic_char atomic_schar atomic_uchar atomic_short atomic_ushort atomic_int atomic_uint atomic_long atomic_ulong atomic_llong atomic_ullong new throw return",built_in:"std string cin cout cerr clog stdin stdout stderr stringstream istringstream ostringstream auto_ptr deque list queue stack vector map set bitset multiset multimap unordered_set unordered_map unordered_multiset unordered_multimap array shared_ptr abort abs acos asin atan2 atan calloc ceil cosh cos exit exp fabs floor fmod fprintf fputs free frexp fscanf isalnum isalpha iscntrl isdigit isgraph islower isprint ispunct isspace isupper isxdigit tolower toupper labs ldexp log10 log malloc realloc memchr memcmp memcpy memset modf pow printf putchar puts scanf sinh sin snprintf sprintf sqrt sscanf strcat strchr strcmp strcpy strcspn strlen strncat strncmp strncpy strpbrk strrchr strspn strstr tanh tan vfprintf vprintf vsprintf endl initializer_list unique_ptr",literal:"true false nullptr NULL"},s=[t,e.CLCM,e.CBCM,n,r];return{aliases:["c","cc","h","c++","h++","hpp"],k:o,i:"</",c:s.concat([a,{b:"\\b(deque|list|queue|stack|vector|map|set|bitset|multiset|multimap|unordered_map|unordered_set|unordered_multiset|unordered_multimap|array)\\s*<",e:">",k:o,c:["self",t]},{b:e.IR+"::",k:o},{v:[{b:/=/,e:/;/},{b:/\(/,e:/\)/},{bK:"new throw return else",e:/;/}],k:o,c:s.concat([{b:/\(/,e:/\)/,k:o,c:s.concat(["self"]),r:0}]),r:0},{cN:"function",b:"("+e.IR+"[\\*&\\s]+)+"+i,rB:!0,e:/[{;=]/,eE:!0,k:o,i:/[^\w\s\*&]/,c:[{b:i,rB:!0,c:[e.TM],r:0},{cN:"params",b:/\(/,e:/\)/,k:o,r:0,c:[e.CLCM,e.CBCM,r,n,t]},e.CLCM,e.CBCM,a]}]),exports:{preprocessor:a,strings:r,k:o}}}),hljs.g("php",function(e){var t={b:"\\$+[a-zA-Z_\x7f-\xff][a-zA-Z0-9_\x7f-\xff]*"},r={cN:"meta",b:/<\?(php)?|\?>/},n={cN:"string",c:[e.BE,r],v:[{b:'b"',e:'"'},{b:"b'",e:"'"},e.inherit(e.ASM,{i:null}),e.inherit(e.QSM,{i:null})]},a={v:[e.BNM,e.CNM]};return{aliases:["php3","php4","php5","php6"],cI:!0,k:"and include_once list abstract global private echo interface as static endswitch array null if endwhile or const for endforeach self var while isset public protected exit foreach throw elseif include __FILE__ empty require_once do xor return parent clone use __CLASS__ __LINE__ else break print eval new catch __METHOD__ case exception default die require __FUNCTION__ enddeclare final try switch continue endfor endif declare unset true false trait goto instanceof insteadof __DIR__ __NAMESPACE__ yield finally",c:[e.HCM,e.C("//","$",{c:[r]}),e.C("/\\*","\\*/",{c:[{cN:"doctag",b:"@[A-Za-z]+"}]}),e.C("__halt_compiler.+?;",!1,{eW:!0,k:"__halt_compiler",l:e.UIR}),{cN:"string",b:/<<<['"]?\w+['"]?$/,e:/^\w+;?$/,c:[e.BE,{cN:"subst",v:[{b:/\$\w+/},{b:/\{\$/,e:/\}/}]}]},r,{cN:"keyword",b:/\$this\b/},t,{b:/(::|->)+[a-zA-Z_\x7f-\xff][a-zA-Z0-9_\x7f-\xff]*/},{cN:"function",bK:"function",e:/[;{]/,eE:!0,i:"\\$|\\[|%",c:[e.UTM,{cN:"params",b:"\\(",e:"\\)",c:["self",t,e.CBCM,n,a]}]},{cN:"class",bK:"class interface",e:"{",eE:!0,i:/[:\(\$"]/,c:[{bK:"extends implements"},e.UTM]},{bK:"namespace",e:";",i:/[\.']/,c:[e.UTM]},{bK:"use",e:";",c:[e.UTM]},{b:"=>"},n,a]}}),hljs.g("css",function(e){var t="[a-zA-Z-][a-zA-Z0-9_-]*",r={b:/[A-Z\_\.\-]+\s*:/,rB:!0,e:";",eW:!0,c:[{cN:"attribute",b:/\S/,e:":",eE:!0,starts:{eW:!0,eE:!0,c:[{b:/[\w-]+\(/,rB:!0,c:[{cN:"built_in",b:/[\w-]+/},{b:/\(/,e:/\)/,c:[e.ASM,e.QSM]}]},e.CSSNM,e.QSM,e.ASM,e.CBCM,{cN:"number",b:"#[0-9A-Fa-f]+"},{cN:"meta",b:"!important"}]}}]};return{cI:!0,i:/[=\/|'\$]/,c:[e.CBCM,{cN:"selector-id",b:/#[A-Za-z0-9_-]+/},{cN:"selector-class",b:/\.[A-Za-z0-9_-]+/},{cN:"selector-attr",b:/\[/,e:/\]/,i:"$"},{cN:"selector-pseudo",b:/:(:)?[a-zA-Z0-9\_\-\+\(\)"'.]+/},{b:"@(font-face|page)",l:"[a-z-]+",k:"font-face page"},{b:"@",e:"[{;]",i:/:/,c:[{cN:"keyword",b:/\w+/},{b:/\s/,eW:!0,eE:!0,r:0,c:[e.ASM,e.QSM,e.CSSNM]}]},{cN:"selector-tag",b:t,r:0},{b:"{",e:"}",i:/\S/,c:[e.CBCM,r]}]}}),hljs.g("apache",function(e){var t={cN:"number",b:"[\\$%]\\d+"};return{aliases:["apacheconf"],cI:!0,c:[e.HCM,{cN:"section",b:"</?",e:">"},{cN:"attribute",b:/\w+/,r:0,k:{nomarkup:"order deny allow setenv rewriterule rewriteengine rewritecond documentroot sethandler errordocument loadmodule options header listen serverroot servername"},starts:{e:/$/,r:0,k:{literal:"on off all"},c:[{cN:"meta",b:"\\s\\[",e:"\\]$"},{cN:"variable",b:"[\\$%]\\{",e:"\\}",c:["self",t]},t,e.QSM]}}],i:/\S/}}),hljs.g("xml",function(e){var t="[A-Za-z0-9\\._:-]+",r={eW:!0,i:/</,r:0,c:[{cN:"attr",b:t,r:0},{b:/=\s*/,r:0,c:[{cN:"string",endsParent:!0,v:[{b:/"/,e:/"/},{b:/'/,e:/'/},{b:/[^\s"'=<>`]+/}]}]}]};return{aliases:["html","xhtml","rss","atom","xjb","xsd","xsl","plist"],cI:!0,c:[{cN:"meta",b:"<!DOCTYPE",e:">",r:10,c:[{b:"\\[",e:"\\]"}]},e.C("<!--","-->",{r:10}),{b:"<\\!\\[CDATA\\[",e:"\\]\\]>",r:10},{b:/<\?(php)?/,e:/\?>/,sL:"php",c:[{b:"/\\*",e:"\\*/",skip:!0}]},{cN:"tag",b:"<style(?=\\s|>|$)",e:">",k:{name:"style"},c:[r],starts:{e:"</style>",rE:!0,sL:["css","xml"]}},{cN:"tag",b:"<script(?=\\s|>|$)",e:">",k:{name:"script"},c:[r],starts:{e:"</script>",rE:!0,sL:["actionscript","javascript","handlebars","xml"]}},{cN:"meta",v:[{b:/<\?xml/,e:/\?>/,r:10},{b:/<\?\w+/,e:/\?>/}]},{cN:"tag",b:"</?",e:"/?>",c:[{cN:"name",b:/[^\/><\s]+/,r:0},r]}]}}),hljs.g("objectivec",function(e){var t={cN:"built_in",b:"\\b(AV|CA|CF|CG|CI|CL|CM|CN|CT|MK|MP|MTK|MTL|NS|SCN|SK|UI|WK|XC)\\w+"},r={keyword:"int float while char export sizeof typedef const struct for union unsigned long volatile static bool mutable if do return goto void enum else break extern asm case short default double register explicit signed typename this switch continue wchar_t inline readonly assign readwrite self @synchronized id typeof nonatomic super unichar IBOutlet IBAction strong weak copy in out inout bycopy byref oneway __strong __weak __block __autoreleasing @private @protected @public @try @property @end @throw @catch @finally @autoreleasepool @synthesize @dynamic @selector @optional @required @encode @package @import @defs @compatibility_alias __bridge __bridge_transfer __bridge_retained __bridge_retain __covariant __contravariant __kindof _Nonnull _Nullable _Null_unspecified __FUNCTION__ __PRETTY_FUNCTION__ __attribute__ getter setter retain unsafe_unretained nonnull nullable null_unspecified null_resettable class instancetype NS_DESIGNATED_INITIALIZER NS_UNAVAILABLE NS_REQUIRES_SUPER NS_RETURNS_INNER_POINTER NS_INLINE NS_AVAILABLE NS_DEPRECATED NS_ENUM NS_OPTIONS NS_SWIFT_UNAVAILABLE NS_ASSUME_NONNULL_BEGIN NS_ASSUME_NONNULL_END NS_REFINED_FOR_SWIFT NS_SWIFT_NAME NS_SWIFT_NOTHROW NS_DURING NS_HANDLER NS_ENDHANDLER NS_VALUERETURN NS_VOIDRETURN",literal:"false true FALSE TRUE nil YES NO NULL",built_in:"BOOL dispatch_once_t dispatch_queue_t dispatch_sync dispatch_async dispatch_once"},n=/[a-zA-Z@][a-zA-Z0-9_]*/,a="@interface @class @protocol @implementation";return{aliases:["mm","objc","obj-c"],k:r,l:n,i:"</",c:[t,e.CLCM,e.CBCM,e.CNM,e.QSM,{cN:"string",v:[{b:'@"',e:'"',i:"\\n",c:[e.BE]},{b:"'",e:"[^\\\\]'",i:"[^\\\\][^']"}]},{cN:"meta",b:"#",e:"$",c:[{cN:"meta-string",v:[{b:'"',e:'"'},{b:"<",e:">"}]}]},{cN:"class",b:"("+a.split(" ").join("|")+")\\b",e:"({|$)",eE:!0,k:a,l:n,c:[e.UTM]},{b:"\\."+e.UIR,r:0}]}}),hljs.g("json",function(e){var t={literal:"true false null"},r=[e.QSM,e.CNM],n={e:",",eW:!0,eE:!0,c:r,k:t},a={b:"{",e:"}",c:[{cN:"attr",b:/"/,e:/"/,c:[e.BE],i:"\\n"},e.inherit(n,{b:/:/})],i:"\\S"},i={b:"\\[",e:"\\]",c:[e.inherit(n)],i:"\\S"};return r.splice(r.length,0,a,i),{c:r,k:t,i:"\\S"}}),hljs.g("coffeescript",function(e){var t={keyword:"in if for while finally new do return else break catch instanceof throw try this switch continue typeof delete debugger super then unless until loop of by when and or is isnt not",literal:"true false null undefined yes no on off",built_in:"npm require console print module global window document"},r="[A-Za-z$_][0-9A-Za-z$_]*",n={cN:"subst",b:/#\{/,e:/}/,k:t},a=[e.BNM,e.inherit(e.CNM,{starts:{e:"(\\s*/)?",r:0}}),{cN:"string",v:[{b:/'''/,e:/'''/,c:[e.BE]},{b:/'/,e:/'/,c:[e.BE]},{b:/"""/,e:/"""/,c:[e.BE,n]},{b:/"/,e:/"/,c:[e.BE,n]}]},{cN:"regexp",v:[{b:"///",e:"///",c:[n,e.HCM]},{b:"//[gim]*",r:0},{b:/\/(?![ *])(\\\/|.)*?\/[gim]*(?=\W|$)/}]},{b:"@"+r},{b:"`",e:"`",eB:!0,eE:!0,sL:"javascript"}];n.c=a;var i=e.inherit(e.TM,{b:r}),o="(\\(.*\\))?\\s*\\B[-=]>",s={cN:"params",b:"\\([^\\(]",rB:!0,c:[{b:/\(/,e:/\)/,k:t,c:["self"].concat(a)}]};return{aliases:["coffee","cson","iced"],k:t,i:/\/\*/,c:a.concat([e.C("###","###"),e.HCM,{cN:"function",b:"^\\s*"+r+"\\s*=\\s*"+o,e:"[-=]>",rB:!0,c:[i,s]},{b:/[:\(,=]\s*/,r:0,c:[{cN:"function",b:o,e:"[-=]>",rB:!0,c:[s]}]},{cN:"class",bK:"class",e:"$",i:/[:="\[\]]/,c:[{bK:"extends",eW:!0,i:/[:="\[\]]/,c:[i]},i]},{b:r+":",e:":",rB:!0,rE:!0,r:0}])}}),hljs.g("javascript",function(e){return{aliases:["js","jsx"],k:{keyword:"in of if for while finally var new function do return void else break catch instanceof with throw case default try this switch continue typeof delete let yield const export super debugger as async await static import from as",literal:"true false null undefined NaN Infinity",built_in:"eval isFinite isNaN parseFloat parseInt decodeURI decodeURIComponent encodeURI encodeURIComponent escape unescape Object Function Boolean Error EvalError InternalError RangeError ReferenceError StopIteration SyntaxError TypeError URIError Number Math Date String RegExp Array Float32Array Float64Array Int16Array Int32Array Int8Array Uint16Array Uint32Array Uint8Array Uint8ClampedArray ArrayBuffer DataView JSON Intl arguments require module console window document Symbol Set Map WeakSet WeakMap Proxy Reflect Promise"},c:[{cN:"meta",r:10,b:/^\s*['"]use (strict|asm)['"]/},{cN:"meta",b:/^#!/,e:/$/},e.ASM,e.QSM,{cN:"string",b:"`",e:"`",c:[e.BE,{cN:"subst",b:"\\$\\{",e:"\\}"}]},e.CLCM,e.CBCM,{cN:"number",v:[{b:"\\b(0[bB][01]+)"},{b:"\\b(0[oO][0-7]+)"},{b:e.CNR}],r:0},{b:"("+e.RSR+"|\\b(case|return|throw)\\b)\\s*",k:"return throw case",c:[e.CLCM,e.CBCM,e.RM,{b:/</,e:/(\/\w+|\w+\/)>/,sL:"xml",c:[{b:/<\w+\s*\/>/,skip:!0},{b:/<\w+/,e:/(\/\w+|\w+\/)>/,skip:!0,c:["self"]}]}],r:0},{cN:"function",bK:"function",e:/\{/,eE:!0,c:[e.inherit(e.TM,{b:/[A-Za-z$_][0-9A-Za-z$_]*/}),{cN:"params",b:/\(/,e:/\)/,eB:!0,eE:!0,c:[e.CLCM,e.CBCM]}],i:/\[|%/},{b:/\$[(.]/},e.METHOD_GUARD,{cN:"class",bK:"class",e:/[{;=]/,eE:!0,i:/[:"\[\]]/,c:[{bK:"extends"},e.UTM]},{bK:"constructor",e:/\{/,eE:!0}],i:/#(?!!)/}}),hljs.g("java",function(e){var t=e.UIR+"(<"+e.UIR+"(\\s*,\\s*"+e.UIR+")*>)?",r="false synchronized int abstract float private char boolean static null if const for true while long strictfp finally protected import native final void enum else break transient catch instanceof byte super volatile case assert short package default double public try this switch continue throws protected public private module requires exports",n="\\b(0[bB]([01]+[01_]+[01]+|[01]+)|0[xX]([a-fA-F0-9]+[a-fA-F0-9_]+[a-fA-F0-9]+|[a-fA-F0-9]+)|(([\\d]+[\\d_]+[\\d]+|[\\d]+)(\\.([\\d]+[\\d_]+[\\d]+|[\\d]+))?|\\.([\\d]+[\\d_]+[\\d]+|[\\d]+))([eE][-+]?\\d+)?)[lLfF]?",a={
cN:"number",b:n,r:0};return{aliases:["jsp"],k:r,i:/<\/|#/,c:[e.C("/\\*\\*","\\*/",{r:0,c:[{b:/\w+@/,r:0},{cN:"doctag",b:"@[A-Za-z]+"}]}),e.CLCM,e.CBCM,e.ASM,e.QSM,{cN:"class",bK:"class interface",e:/[{;=]/,eE:!0,k:"class interface",i:/[:"\[\]]/,c:[{bK:"extends implements"},e.UTM]},{bK:"new throw return else",r:0},{cN:"function",b:"("+t+"\\s+)+"+e.UIR+"\\s*\\(",rB:!0,e:/[{;=]/,eE:!0,k:r,c:[{b:e.UIR+"\\s*\\(",rB:!0,r:0,c:[e.UTM]},{cN:"params",b:/\(/,e:/\)/,k:r,r:0,c:[e.ASM,e.QSM,e.CNM,e.CBCM]},e.CLCM,e.CBCM]},a,{cN:"meta",b:"@[A-Za-z]+"}]}}),hljs.g("tex",function(e){var t={cN:"tag",b:/\\/,r:0,c:[{cN:"name",v:[{b:/[a-zA-Z\u0430-\u044f\u0410-\u044f]+[*]?/},{b:/[^a-zA-Z\u0430-\u044f\u0410-\u044f0-9]/}],starts:{eW:!0,r:0,c:[{cN:"string",v:[{b:/\[/,e:/\]/},{b:/\{/,e:/\}/}]},{b:/\s*=\s*/,eW:!0,r:0,c:[{cN:"number",b:/-?\d*\.?\d+(pt|pc|mm|cm|in|dd|cc|ex|em)?/}]}]}}]};return{c:[t,{cN:"formula",c:[t],r:0,v:[{b:/\$\$/,e:/\$\$/},{b:/\$/,e:/\$/}]},e.C("%","$",{r:0})]}}),hljs.g("python",function(e){var t={cN:"meta",b:/^(>>>|\.\.\.) /},r={cN:"string",c:[e.BE],v:[{b:/(u|b)?r?'''/,e:/'''/,c:[t],r:10},{b:/(u|b)?r?"""/,e:/"""/,c:[t],r:10},{b:/(u|r|ur)'/,e:/'/,r:10},{b:/(u|r|ur)"/,e:/"/,r:10},{b:/(b|br)'/,e:/'/},{b:/(b|br)"/,e:/"/},e.ASM,e.QSM]},n={cN:"number",r:0,v:[{b:e.BNR+"[lLjJ]?"},{b:"\\b(0o[0-7]+)[lLjJ]?"},{b:e.CNR+"[lLjJ]?"}]},a={cN:"params",b:/\(/,e:/\)/,c:["self",t,n,r]};return{aliases:["py","gyp"],k:{keyword:"and elif is global as in if from raise for except finally print import pass return exec else break not with class assert yield try while continue del or def lambda async await nonlocal|10 None True False",built_in:"Ellipsis NotImplemented"},i:/(<\/|->|\?)/,c:[t,n,r,e.HCM,{v:[{cN:"function",bK:"def",r:10},{cN:"class",bK:"class"}],e:/:/,i:/[${=;\n,]/,c:[e.UTM,a,{b:/->/,eW:!0,k:"None"}]},{cN:"meta",b:/^[\t ]*@/,e:/$/},{b:/\b(print|exec)\(/}]}}),hljs.g("ini",function(e){var t={cN:"string",c:[e.BE],v:[{b:"'''",e:"'''",r:10},{b:'"""',e:'"""',r:10},{b:'"',e:'"'},{b:"'",e:"'"}]};return{aliases:["toml"],cI:!0,i:/\S/,c:[e.C(";","$"),e.HCM,{cN:"section",b:/^\s*\[+/,e:/\]+/},{b:/^[a-z0-9\[\]_-]+\s*=\s*/,e:"$",rB:!0,c:[{cN:"attr",b:/[a-z0-9\[\]_-]+/},{b:/=/,eW:!0,r:0,c:[{cN:"literal",b:/\bon|off|true|false|yes|no\b/},{cN:"variable",v:[{b:/\$[\w\d"][\w\d_]*/},{b:/\$\{(.*?)}/}]},t,{cN:"number",b:/([\+\-]+)?[\d]+_[\d_]+/},e.NM]}]}]}}),hljs.g("http",function(e){var t="HTTP/[0-9\\.]+";return{aliases:["https"],i:"\\S",c:[{b:"^"+t,e:"$",c:[{cN:"number",b:"\\b\\d{3}\\b"}]},{b:"^[A-Z]+ (.*?) "+t+"$",rB:!0,e:"$",c:[{cN:"string",b:" ",e:" ",eB:!0,eE:!0},{b:t},{cN:"keyword",b:"[A-Z]+"}]},{cN:"attribute",b:"^\\w",e:": ",eE:!0,i:"\\n|\\s|=",starts:{e:"$",r:0}},{b:"\\n\\n",starts:{sL:[],eW:!0}}]}}),hljs.g("ruby",function(e){var t="[a-zA-Z_]\\w*[!?=]?|[-+~]\\@|<<|>>|=~|===?|<=>|[<>]=?|\\*\\*|[-/+%^&*~`|]|\\[\\]=?",r={keyword:"and then defined module in return redo if BEGIN retry end for self when next until do begin unless END rescue else break undef not super class case require yield alias while ensure elsif or include attr_reader attr_writer attr_accessor",literal:"true false nil"},n={cN:"doctag",b:"@[A-Za-z]+"},a={b:"#<",e:">"},i=[e.C("#","$",{c:[n]}),e.C("^\\=begin","^\\=end",{c:[n],r:10}),e.C("^__END__","\\n$")],o={cN:"subst",b:"#\\{",e:"}",k:r},s={cN:"string",c:[e.BE,o],v:[{b:/'/,e:/'/},{b:/"/,e:/"/},{b:/`/,e:/`/},{b:"%[qQwWx]?\\(",e:"\\)"},{b:"%[qQwWx]?\\[",e:"\\]"},{b:"%[qQwWx]?{",e:"}"},{b:"%[qQwWx]?<",e:">"},{b:"%[qQwWx]?/",e:"/"},{b:"%[qQwWx]?%",e:"%"},{b:"%[qQwWx]?-",e:"-"},{b:"%[qQwWx]?\\|",e:"\\|"},{b:/\B\?(\\\d{1,3}|\\x[A-Fa-f0-9]{1,2}|\\u[A-Fa-f0-9]{4}|\\?\S)\b/}]},c={cN:"params",b:"\\(",e:"\\)",endsParent:!0,k:r},l=[s,a,{cN:"class",bK:"class module",e:"$|;",i:/=/,c:[e.inherit(e.TM,{b:"[A-Za-z_]\\w*(::\\w+)*(\\?|\\!)?"}),{b:"<\\s*",c:[{b:"("+e.IR+"::)?"+e.IR}]}].concat(i)},{cN:"function",bK:"def",e:"$|;",c:[e.inherit(e.TM,{b:t}),c].concat(i)},{b:e.IR+"::"},{cN:"symbol",b:e.UIR+"(\\!|\\?)?:",r:0},{cN:"symbol",b:":(?!\\s)",c:[s,{b:t}],r:0},{cN:"number",b:"(\\b0[0-7_]+)|(\\b0x[0-9a-fA-F_]+)|(\\b[1-9][0-9_]*(\\.[0-9_]+)?)|[0_]\\b",r:0},{b:"(\\$\\W)|((\\$|\\@\\@?)(\\w+))"},{cN:"params",b:/\|/,e:/\|/,k:r},{b:"("+e.RSR+")\\s*",c:[a,{cN:"regexp",c:[e.BE,o],i:/\n/,v:[{b:"/",e:"/[a-z]*"},{b:"%r{",e:"}[a-z]*"},{b:"%r\\(",e:"\\)[a-z]*"},{b:"%r!",e:"![a-z]*"},{b:"%r\\[",e:"\\][a-z]*"}]}].concat(i),r:0}].concat(i);o.c=l,c.c=l;var u="[>?]>",d="[\\w#]+\\(\\w+\\):\\d+:\\d+>",g="(\\w+-)?\\d+\\.\\d+\\.\\d(p\\d+)?[^>]+>",p=[{b:/^\s*=>/,starts:{e:"$",c:l}},{cN:"meta",b:"^("+u+"|"+d+"|"+g+")",starts:{e:"$",c:l}}];return{aliases:["rb","gemspec","podspec","thor","irb"],k:r,i:/\/\*/,c:i.concat(p).concat(l)}}),hljs.g("sql",function(e){var t=e.C("--","$");return{cI:!0,i:/[<>{}*#]/,c:[{bK:"begin end start commit rollback savepoint lock alter create drop rename call delete do handler insert load replace select truncate update set show pragma grant merge describe use explain help declare prepare execute deallocate release unlock purge reset change stop analyze cache flush optimize repair kill install uninstall checksum restore check backup revoke",e:/;/,eW:!0,l:/[\w\.]+/,k:{keyword:"",literal:"true false null",built_in:"array bigint binary bit blob boolean char character date dec decimal float int int8 integer interval number numeric real record serial serial8 smallint text varchar varying void"},c:[{cN:"string",b:"'",e:"'",c:[e.BE,{b:"''"}]},{cN:"string",b:'"',e:'"',c:[e.BE,{b:'""'}]},{cN:"string",b:"`",e:"`",c:[e.BE]},e.CNM,e.CBCM,t]},e.CBCM,t]}}),hljs.g("makefile",function(e){var t={cN:"variable",b:/\$\(/,e:/\)/,c:[e.BE]};return{aliases:["mk","mak"],c:[e.HCM,{b:/^\w+\s*\W*=/,rB:!0,r:0,starts:{e:/\s*\W*=/,eE:!0,starts:{e:/$/,r:0,c:[t]}}},{cN:"section",b:/^[\w]+:\s*$/},{cN:"meta",b:/^\.PHONY:/,e:/$/,k:{"meta-keyword":".PHONY"},l:/[\.\w]+/},{b:/^\t+/,e:/$/,r:0,c:[e.QSM,t]}]}}),hljs.g("perl",function(e){var t="getpwent getservent quotemeta msgrcv scalar kill dbmclose undef lc ma syswrite tr send umask sysopen shmwrite vec qx utime local oct semctl localtime readpipe do return format read sprintf dbmopen pop getpgrp not getpwnam rewinddir qqfileno qw endprotoent wait sethostent bless s|0 opendir continue each sleep endgrent shutdown dump chomp connect getsockname die socketpair close flock exists index shmgetsub for endpwent redo lstat msgctl setpgrp abs exit select print ref gethostbyaddr unshift fcntl syscall goto getnetbyaddr join gmtime symlink semget splice x|0 getpeername recv log setsockopt cos last reverse gethostbyname getgrnam study formline endhostent times chop length gethostent getnetent pack getprotoent getservbyname rand mkdir pos chmod y|0 substr endnetent printf next open msgsnd readdir use unlink getsockopt getpriority rindex wantarray hex system getservbyport endservent int chr untie rmdir prototype tell listen fork shmread ucfirst setprotoent else sysseek link getgrgid shmctl waitpid unpack getnetbyname reset chdir grep split require caller lcfirst until warn while values shift telldir getpwuid my getprotobynumber delete and sort uc defined srand accept package seekdir getprotobyname semop our rename seek if q|0 chroot sysread setpwent no crypt getc chown sqrt write setnetent setpriority foreach tie sin msgget map stat getlogin unless elsif truncate exec keys glob tied closedirioctl socket readlink eval xor readline binmode setservent eof ord bind alarm pipe atan2 getgrent exp time push setgrent gt lt or ne m|0 break given say state when",r={cN:"subst",b:"[$@]\\{",e:"\\}",k:t},n={b:"->{",e:"}"},a={v:[{b:/\$\d/},{b:/[\$%@](\^\w\b|#\w+(::\w+)*|{\w+}|\w+(::\w*)*)/},{b:/[\$%@][^\s\w{]/,r:0}]},i=[e.BE,r,a],o=[a,e.HCM,e.C("^\\=\\w","\\=cut",{eW:!0}),n,{cN:"string",c:i,v:[{b:"q[qwxr]?\\s*\\(",e:"\\)",r:5},{b:"q[qwxr]?\\s*\\[",e:"\\]",r:5},{b:"q[qwxr]?\\s*\\{",e:"\\}",r:5},{b:"q[qwxr]?\\s*\\|",e:"\\|",r:5},{b:"q[qwxr]?\\s*\\<",e:"\\>",r:5},{b:"qw\\s+q",e:"q",r:5},{b:"'",e:"'",c:[e.BE]},{b:'"',e:'"'},{b:"`",e:"`",c:[e.BE]},{b:"{\\w+}",c:[],r:0},{b:"-?\\w+\\s*\\=\\>",c:[],r:0}]},{cN:"number",b:"(\\b0[0-7_]+)|(\\b0x[0-9a-fA-F_]+)|(\\b[1-9][0-9_]*(\\.[0-9_]+)?)|[0_]\\b",r:0},{b:"(\\/\\/|"+e.RSR+"|\\b(split|return|print|reverse|grep)\\b)\\s*",k:"split return print reverse grep",r:0,c:[e.HCM,{cN:"regexp",b:"(s|tr|y)/(\\\\.|[^/])*/(\\\\.|[^/])*/[a-z]*",r:10},{cN:"regexp",b:"(m|qr)?/",e:"/[a-z]*",c:[e.BE],r:0}]},{cN:"function",bK:"sub",e:"(\\s*\\(.*?\\))?[;{]",eE:!0,r:5,c:[e.TM]},{b:"-\\w\\b",r:0},{b:"^__DATA__$",e:"^__END__$",sL:"mojolicious",c:[{b:"^@@.*",e:"$",cN:"comment"}]}];return r.c=o,n.c=o,{aliases:["pl","pm"],l:/[\w\.]+/,k:t,c:o}}),hljs.g("cs",function(e){var t={keyword:"abstract as base bool break byte case catch char checked const continue decimal dynamic default delegate do double else enum event explicit extern finally fixed float for foreach goto if implicit in int interface internal is lock long when object operator out override params private protected public readonly ref sbyte sealed short sizeof stackalloc static string struct switch this try typeof uint ulong unchecked unsafe ushort using virtual volatile void while async nameof ascending descending from get group into join let orderby partial select set value var where yield",literal:"null false true"},r={cN:"string",b:'@"',e:'"',c:[{b:'""'}]},n=e.inherit(r,{i:/\n/}),a={cN:"subst",b:"{",e:"}",k:t},i=e.inherit(a,{i:/\n/}),o={cN:"string",b:/\$"/,e:'"',i:/\n/,c:[{b:"{{"},{b:"}}"},e.BE,i]},s={cN:"string",b:/\$@"/,e:'"',c:[{b:"{{"},{b:"}}"},{b:'""'},a]},c=e.inherit(s,{i:/\n/,c:[{b:"{{"},{b:"}}"},{b:'""'},i]});a.c=[s,o,r,e.ASM,e.QSM,e.CNM,e.CBCM],i.c=[c,o,n,e.ASM,e.QSM,e.CNM,e.inherit(e.CBCM,{i:/\n/})];var l={v:[s,o,r,e.ASM,e.QSM]},u=e.IR+"(<"+e.IR+">)?(\\[\\])?";return{aliases:["csharp"],k:t,i:/::/,c:[e.C("///","$",{rB:!0,c:[{cN:"doctag",v:[{b:"///",r:0},{b:"<!--|-->"},{b:"</?",e:">"}]}]}),e.CLCM,e.CBCM,{cN:"meta",b:"#",e:"$",k:{"meta-keyword":"if else elif endif define undef warning error line region endregion pragma checksum"}},l,e.CNM,{bK:"class interface",e:/[{;=]/,i:/[^\s:]/,c:[e.TM,e.CLCM,e.CBCM]},{bK:"namespace",e:/[{;=]/,i:/[^\s:]/,c:[e.inherit(e.TM,{b:"[a-zA-Z](\\.?\\w)*"}),e.CLCM,e.CBCM]},{bK:"new return throw await",r:0},{cN:"function",b:"("+u+"\\s+)+"+e.IR+"\\s*\\(",rB:!0,e:/[{;=]/,eE:!0,k:t,c:[{b:e.IR+"\\s*\\(",rB:!0,c:[e.TM],r:0},{cN:"params",b:/\(/,e:/\)/,eB:!0,eE:!0,k:t,r:0,c:[l,e.CNM,e.CBCM]},e.CLCM,e.CBCM]}]}});var R="Menlo,Consolas,monospace",U=105.1316178/t(R)+"px",z=e("style",'body{max-width:680px;margin:auto;padding:20px;text-align:justify;line-height:140%; -webkit-font-smoothing:antialiased;-moz-osx-font-smoothing:grayscale;font-smoothing:antialiased;color:#222;font-family:Palatino,Georgia,"Times New Roman",serif}'),O=e("style","body{counter-reset: h1 h2 h3 h4 h5 h6}.md code,pre{font-family:"+R+";font-size:"+U+';line-height:140%}.md div.title{font-size:26px;font-weight:800;line-height:120%;text-align:center}.md div.afterTitles{height:10px}.md div.subtitle{text-align:center}.md .image{display:inline-block}.md div.imagecaption,.md div.tablecaption,.md div.listingcaption{margin:0.2em 5px 10px 5px;text-align: justify;font-style:italic}.md div.imagecaption{margin-bottom:0}.md img{max-width:100%;page-break-inside:avoid}li{text-align:left};.md div.tilde{margin:20px 0 -10px;text-align:center}.md blockquote.fancyquote{margin:25px 0 25px;text-align:left;line-height:160%}.md blockquote.fancyquote::before{content:"\u201c";color:#DDD;font-family:Times New Roman;font-size:45px;line-height:0;margin-right:6px;vertical-align:-0.3em}.md span.fancyquote{font-size:118%;color:#777;font-style:italic}.md span.fancyquote::after{content:"\u201d";font-style:normal;color:#DDD;font-family:Times New Roman;font-size:45px;line-height:0;margin-left:6px;vertical-align:-0.3em}.md blockquote.fancyquote .author{width:100%;margin-top:10px;display:inline-block;text-align:right}.md small{font-size:60%}.md div.title,contents,.md .tocHeader,h1,h2,h3,h4,h5,h6,.md .shortTOC,.md .mediumTOC,.nonumberh1,.nonumberh2,.nonumberh3,.nonumberh4,.nonumberh5,.nonumberh6{font-family:Verdana,Helvetica,Arial,sans-serif;margin:13.4px 0 13.4px;padding:15px 0 3px;border-top:none;clear:both}.md svg.diagram{display:block;font-family:'+R+";font-size:"+U+";text-align:center;stroke-linecap:round;stroke-width:"+L+"px;page-break-inside:avoid;stroke:#000;fill:#000}.md svg.diagram .opendot{fill:#FFF}.md svg.diagram text{stroke:none}.md a{font-family:Georgia,Palatino,'Times New Roman'}h1,.tocHeader,.nonumberh1{border-bottom:3px solid;font-size:20px;font-weight:bold;}h1,.nonumberh1{counter-reset: h2 h3 h4 h5 h6}h2,.nonumberh2{counter-reset: h3 h4 h5 h6;border-bottom:2px solid #999;color:#555;font-size:18px;}h3,h4,h5,h6,.nonumberh3,.nonumberh4,.nonumberh5,.nonumberh6{font-family:Helvetica,Arial,sans-serif;color:#555;font-size:16px;}h3{counter-reset:h4 h5 h6}h4{counter-reset:h5 h6}h5{counter-reset:h6}.md table{border-collapse:collapse;line-height:140%;page-break-inside:avoid}.md table.table{margin:auto}.md table.calendar{width:100%;margin:auto;font-size:11px;font-family:Helvetica,Arial,sans-serif}.md table.calendar th{font-size:16px}.md .today{background:#ECF8FA}.md .calendar .parenthesized{color:#999;font-style:italic}.md div.tablecaption{text-align:center}.md table.table th{color:#FFF;background-color:#AAA;border:1px solid #888;padding:8px 15px 8px 15px}.md table.table td{padding:5px 15px 5px 15px;border:1px solid #888}.md table.table tr:nth-child(even){background:#EEE}.md pre.tilde{border-top: 1px solid #CCC;border-bottom: 1px solid #CCC;padding: 5px 0 5px 20px;margin:0 0 30px 0;background:#FCFCFC;page-break-inside:avoid}.md a:link, .md a:visited{color:#38A;text-decoration:none}.md a:link:hover{text-decoration:underline}.md dt{font-weight:700}dl>.md dd{padding:0 0 18px}.md dl>table{margin:35px 0 30px}.md code{white-space:pre;page-break-inside:avoid}.md .endnote{font-size:13px;line-height:15px;padding-left:10px;text-indent:-10px}.md .bib{padding-left:80px;text-indent:-80px;text-align:left}.markdeepFooter{font-size:9px;text-align:right;padding-top:80px;color:#999}.md .mediumTOC{float:right;font-size:12px;line-height:15px;border-left:1px solid #CCC;padding-left:15px;margin:15px 0px 15px 25px}.md .mediumTOC .level1{font-weight:600}.md .longTOC .level1{font-weight:600;display:block;padding-top:12px;margin:0 0 -20px}.md .shortTOC{text-align:center;font-weight:bold;margin-top:15px;font-size:14px}"),F='<!-- Markdeep: --><style class="fallback">body{visibility:hidden;white-space:pre;font-family:monospace}</style><script src="markdeep.min.js"></script><script src="https://casual-effects.com/markdeep/latest/markdeep.min.js"></script><script>window.alreadyProcessedMarkdeep||(document.body.style.visibility="visible")</script>',H={keyword:{table:"tableau",figure:"figure",m:"liste",diagram:"diagramme",contents:"Table des mati\xe8res",sec:"sec",section:"section",subsection:"paragraphe",Monday:"lundi",Tuesday:"mardi",Wednesday:"mercredi",Thursday:"jeudi",Friday:"vendredi",Saturday:"samedi",Sunday:"dimanche",January:"Janvier",February:"F\xe9vrier",March:"Mars",April:"Avril",May:"Mai",June:"Juin",July:"Julliet",August:"Ao\xfbt",September:"Septembre",October:"Octobre",November:"Novembre",December:"D\xe9cembre",jan:"janv",feb:"f\xe9vr",mar:"mars",apr:"avril",may:"mai",jun:"juin",jul:"juil",aug:"ao\xfbt",sep:"sept",oct:"oct",nov:"nov",dec:"d\xe9c"}},q={keyword:{table:"\u0442\u0430\u0431\u043b\u0438\u0446\u0430",figure:"\u0444\u0438\u0433\u0443\u0440\u0430",m:"\u0441\u043f\u0438\u0441\u044a\u043a",diagram:"\u0434\u0438\u0430\u0433\u0440\u0430\u043c\u0430",contents:"c\u044a\u0434\u044a\u0440\u0436\u0430\u043d\u0438\u0435",sec:"\u0441\u0435\u043a",section:"\u0440\u0430\u0437\u0434\u0435\u043b",subsection:"\u043f\u043e\u0434\u0440\u0430\u0437\u0434\u0435\u043b",Monday:"\u043f\u043e\u043d\u0435\u0434\u0435\u043b\u043d\u0438\u043a",Tuesday:"\u0432\u0442\u043e\u0440\u043d\u0438\u043a",Wednesday:"\u0441\u0440\u044f\u0434\u0430",Thursday:"\u0447\u0435\u0442\u0432\u044a\u0440\u0442\u044a\u043a",Friday:"\u043f\u0435\u0442\u044a\u043a",Saturday:"\u0441\u044a\u0431\u043e\u0442\u0430",Sunday:"\u043d\u0435\u0434\u0435\u043b\u044f",January:"\u044f\u043d\u0443\u0430\u0440\u0438",February:"\u0444\u0435\u0432\u0440\u0443\u0430\u0440\u0438",March:"\u043c\u0430\u0440\u0442",April:"\u0430\u043f\u0440\u0438\u043b",May:"\u043c\u0430\u0439",June:"\u044e\u043d\u0438",July:"\u044e\u043b\u0438",August:"\u0430\u0432\u0433\u0443\u0441\u0442",September:"\u0441\u0435\u043f\u0442\u0435\u043c\u0432\u0440\u0438",October:"\u043e\u043a\u0442\u043e\u043c\u0432\u0440\u0438",November:"\u043d\u043e\u0435\u043c\u0432\u0440\u0438",December:"\u0434\u0435\u043a\u0435\u043c\u0432\u0440\u0438",jan:"\u044f\u043d",feb:"\u0444\u0435\u0432\u0440",mar:"\u043c\u0430\u0440\u0442",apr:"\u0430\u043f\u0440",may:"\u043c\u0430\u0439",jun:"\u044e\u043d\u0438",jul:"\u044e\u043b\u0438",aug:"\u0430\u0432\u0433",sep:"\u0441\u0435\u043f\u0442",oct:"\u043e\u043a\u0442",nov:"\u043d\u043e\u0435\u043c",dec:"\u0434\u0435\u043a"}},P={keyword:{table:"\u0442\u0430\u0431\u043b\u0438\u0446\u0430",figure:"\u0440\u0438\u0441\u0443\u043d\u043e\u043a",m:"\u043b\u0438\u0441\u0442\u0438\u043d\u0433",diagram:"\u0434\u0438\u0430\u0433\u0440\u0430\u043c\u043c\u0430",contents:"\u0421\u043e\u0434\u0435\u0440\u0436\u0430\u043d\u0438\u0435",sec:"\u0441\u0435\u043a",section:"\u0440\u0430\u0437\u0434\u0435\u043b",subsection:"\u043f\u043e\u0434\u0440\u0430\u0437\u0434\u0435\u043b",Monday:"\u043f\u043e\u043d\u0435\u0434\u0435\u043b\u044c\u043d\u0438\u043a",Tuesday:"\u0432\u0442\u043e\u0440\u043d\u0438\u043a",Wednesday:"\u0441\u0440\u0435\u0434\u0430",Thursday:"\u0447\u0435\u0442\u0432\u0435\u0440\u0433",Friday:"\u043f\u044f\u0442\u043d\u0438\u0446\u0430",Saturday:"\u0441\u0443\u0431\u0431\u043e\u0442\u0430",Sunday:"\u0432\u043e\u0441\u043a\u0440\u0435\u0441\u0435\u043d\u044c\u0435",January:"\u044f\u043d\u0432\u0430\u0440\u044cr",February:"\u0444\u0435\u0432\u0440\u0430\u043b\u044c",March:"\u043c\u0430\u0440\u0442",April:"\u0430\u043f\u0440\u0435\u043b\u044c",May:"\u043c\u0430\u0439",June:"\u0438\u044e\u043d\u044c",July:"\u0438\u044e\u043b\u044c",August:"\u0430\u0432\u0433\u0443\u0441\u0442",September:"\u0441\u0435\u043d\u0442\u044f\u0431\u0440\u044c",October:"\u043e\u043a\u0442\u044f\u0431\u0440\u044c",November:"\u043d\u043e\u044f\u0431\u0440\u044c",December:"\u0434\u0435\u043a\u0430\u0431\u0440\u044c",jan:"\u044f\u043d\u0432",feb:"\u0444\u0435\u0432\u0440",mar:"\u043c\u0430\u0440\u0442",apr:"\u0430\u043f\u0440",may:"\u043c\u0430\u0439",jun:"\u0438\u044e\u043d\u044c",jul:"\u0438\u044e\u043b\u044c",aug:"\u0430\u0432\u0433",sep:"\u0441\u0435\u043d\u0442",oct:"\u043e\u043a\u0442",nov:"\u043d\u043e\u044f\u0431\u0440\u044c",dec:"\u0434\u0435\u043a"}},V={keyword:{table:"t\xe1bl\xe1zat",figure:"\xe1bra",m:"lista",diagram:"diagramm",contents:"Tartalomjegyz\xe9k",sec:"fej",section:"fejezet",subsection:"alfejezet",Monday:"h\xe9tf\u0151",Tuesday:"kedd",Wednesday:"szerda",Thursday:"cs\xfct\xf6rt\xf6k",Friday:"p\xe9ntek",Saturday:"szombat",Sunday:"vas\xe1rnap",January:"janu\xe1r",February:"febru\xe1r",March:"m\xe1rcius",April:"\xe1prilis",May:"m\xe1jus",June:"j\xfanius",July:"j\xfalius",August:"augusztus",September:"szeptember",October:"okt\xf3ber",November:"november",December:"december",jan:"jan",feb:"febr",mar:"m\xe1rc",apr:"\xe1pr",may:"m\xe1j",jun:"j\xfan",jul:"j\xfal",aug:"aug",sep:"szept",oct:"okt",nov:"nov",dec:"dec"}},W={keyword:{table:"Tabelle",figure:"Abbildung",m:"Auflistung",diagram:"Diagramm",contents:"Inhaltsverzeichnis",sec:"Kap",section:"Kapitel",subsection:"Unterabschnitt",Monday:"Montag",Tuesday:"Dienstag",Wednesday:"Mittwoch",Thursday:"Donnerstag",Friday:"Freitag",Saturday:"Samstag",Sunday:"Sonntag",January:"Januar",February:"Februar",March:"M\xe4rz",April:"April",May:"Mai",June:"Juni",July:"Juli",August:"August",September:"September",October:"Oktober",November:"November",December:"Dezember",jan:"Jan",feb:"Feb",mar:"M\xe4r",apr:"Apr",may:"Mai",jun:"Jun",jul:"Jul",aug:"Aug",sep:"Sep",oct:"Okt",nov:"Nov",dec:"Dez"}},Z={mode:"markdeep",detectMath:!0,lang:{keyword:{}},tocStyle:"auto",hideEmptyWeekends:!0,showURLs:!1,o:!0},G={ru:P,fr:H,bg:q,de:W,hu:V};[].slice.call(document.getElementsByTagName("meta")).forEach(function(e){var t=e.getAttribute("lang");if(t){var r=G[t];r&&(Z.lang=r)}});var J=Math.max,K=Math.min,Q=Math.abs,X=Math.sign||function(e){return+e===e?0===e?e:e>0?1:-1:NaN},Y="<style>.hljs{display:block;overflow-x:auto;padding:0.5em;background:#fff;color:#000;-webkit-text-size-adjust:none}.hljs-comment{color:#006a00}.hljs-keyword{color:#02E}.hljs-literal,.nginx .hljs-title{color:#aa0d91}.method,.hljs-list .hljs-title,.hljs-tag .hljs-title,.setting .hljs-value,.hljs-winutils,.tex .hljs-command,.http .hljs-title,.hljs-request,.hljs-status,.hljs-name{color:#008}.hljs-envvar,.tex .hljs-special{color:#660}.hljs-string{color:#c41a16}.hljs-tag .hljs-value,.hljs-cdata,.hljs-filter .hljs-argument,.hljs-attr_selector,.apache .hljs-cbracket,.hljs-date,.hljs-regexp{color:#080}.hljs-sub .hljs-identifier,.hljs-pi,.hljs-tag,.hljs-tag .hljs-keyword,.hljs-decorator,.ini .hljs-title,.hljs-shebang,.hljs-prompt,.hljs-hexcolor,.hljs-rule .hljs-value,.hljs-symbol,.hljs-symbol .hljs-string,.hljs-number,.css .hljs-function,.hljs-function .hljs-title,.coffeescript .hljs-attribute{color:#A0C}.hljs-function .hljs-title{font-weight:bold;color:#000}.hljs-class .hljs-title,.smalltalk .hljs-class,.hljs-type,.hljs-typename,.hljs-tag .hljs-attribute,.hljs-doctype,.hljs-class .hljs-id,.hljs-built_in,.setting,.hljs-params,.clojure .hljs-attribute{color:#5c2699}.hljs-variable{color:#3f6e74}.css .hljs-tag,.hljs-rule .hljs-property,.hljs-pseudo,.hljs-subst{color:#000}.css .hljs-class,.css .hljs-id{color:#9b703f}.hljs-value .hljs-important{color:#ff7700;font-weight:bold}.hljs-rule .hljs-keyword{color:#c5af75}.hljs-annotation,.apache .hljs-sqbracket,.nginx .hljs-built_in{color:#9b859d}.hljs-preprocessor,.hljs-preprocessor *,.hljs-pragma{color:#643820}.tex .hljs-formula{background-color:#eee;font-style:italic}.diff .hljs-header,.hljs-chunk{color:#808080;font-weight:bold}.diff .hljs-change{background-color:#bccff9}.hljs-addition{background-color:#baeeba}.hljs-deletion{background-color:#ffc8bd}.hljs-comment .hljs-doctag{font-weight:bold}.method .hljs-id{color:#000}</style>";if(!window.alreadyProcessedMarkdeep){window.alreadyProcessedMarkdeep=!0;var ee=window.location.href.search(/\?.*noformat.*/i)!==-1;window.markdeep=Object.freeze({format:v,formatDiagram:M,stylesheet:function(){return O+l()+Y}});var te=r("mode");switch(te){case"script":return;case"html":case"doxygen":return k(document.getElementsByClassName("diagram")).concat(k(document.getElementsByTagName("diagram"))).forEach(function(e){var t=o(e.innerHTML);t=t.rp(/(:?^[ \t]*\n)|(:?\n[ \t]*)$/g,""),"doxygen"===te&&(t=t.rp(RegExp("\u2013","g"),"--"),t=t.rp(RegExp("\u2014","g"),"---"),t=t.rp(/<a class="el" .*>(.*)<\/a>/g,"$1")),e.outerHTML='<center class="md">'+M(w(t),"")+"</center>"}),k(document.getElementsByClassName("markdeep")).concat(k(document.getElementsByTagName("markdeep"))).forEach(function(e){var t=document.createElement("div");t.innerHTML=v(w(o(e.innerHTML)),!0),e.parentNode.replaceChild(t,e)}),void(document.head.innerHTML=window.markdeep.stylesheet()+document.head.innerHTML)}ee||(k(document.getElementsByTagName("script")).forEach(function(e){N(e.src)&&e.parentNode.removeChild(e)}),document.body.style.visibility="hidden");var re=u(document.body);if(ee){re=re.rp(/<!-- Markdeep:.+$/gm,"")+F,re=re.rp(/</g,"&lt;").rp(/>/g,"&gt;"),document.body.innerHTML=e("pre",re);for(var ne=document.getElementsByClassName("fallback"),ae=0;ae<ne.length;++ae)ne[ae].remove();return}var ie=function(){var t=u(document.body);t=o(t);var n=v(t,!1),a=r("detectMath")&&(n.search(/(?:\$\$[\s\S]+\$\$)|(?:\\begin{)/m)!==-1||n.search(/\\\(.*\\\)/)!==-1);if(a){var s="$$NC{\\n}{\\hat{n}}NC{\\w}{\\hat{\\omega}}NC{\\wi}{\\w_\\mathrm{i}}NC{\\wo}{\\w_\\mathrm{o}}NC{\\wh}{\\w_\\mathrm{h}}NC{\\Li}{L_\\mathrm{i}}NC{\\Lo}{L_\\mathrm{o}}NC{\\Le}{L_\\mathrm{e}}NC{\\Lr}{L_\\mathrm{r}}NC{\\Lt}{L_\\mathrm{t}}NC{\\O}{\\mathrm{O}}NC{\\degrees}{{^\\circ}}NC{\\T}{\\mathsf{T}}NC{\\mathset}[1]{\\mathbb{#1}}NC{\\Real}{\\mathset{R}}NC{\\Integer}{\\mathset{Z}}NC{\\Boolean}{\\mathset{B}}NC{\\Complex}{\\mathset{C}}$$\n".rp(/NC/g,"\\newcommand");n='<script type="text/x-mathjax-config">MathJax.Hub.Config({ TeX: { equationNumbers: {autoNumber: "AMS"} } });</script><span style="display:none">'+s+"</span>\n"+n}n+=T;var c=t.length>1e3,d=z+O+l()+Y;if(c&&(d+=e("style","div.title { padding-top: 40px; } div.afterTitles { height: 15px; }")),window.location.href.search(/\?.*export.*/i)!==-1){var g='<meta charset="UTF-8"><meta http-equiv="content-type" content="text/html;charset=UTF-8">'+d+document.head.innerHTML+n;a&&(g+='<script src="https://cdn.mathjax.org/mathjax/latest/MathJax.js?config=TeX-AMS-MML_HTMLorMML"></script>'),document.body.innerHTML=e("code",i(g))}else if(document.head.innerHTML='<meta charset="UTF-8"><meta http-equiv="content-type" content="text/html;charset=UTF-8">'+d+document.head.innerHTML,document.body.innerHTML=n,a){var p=document.createElement("script");p.type="text/javascript",p.src="https://cdn.mathjax.org/mathjax/latest/MathJax.js?config=TeX-AMS-MML_HTMLorMML",document.getElementsByTagName("head")[0].appendChild(p)}document.body.style.visibility="visible"},oe=/([^?]+)(?:\?id=(inc\d+)&p=([^&]+))?/.exec(location.href),se=A(oe[1]),ce=oe[2],le=(A(oe[3]&&decodeURIComponent(oe[3])),"display:none"),ue=0,de=ce,ge=!1,pe=0;re=re.rp(/(?:^|\s)\(insert[ \t]+(\S+\.\S*)[ \t]+here\)\s/g,function(e,t){0===pe&&(ge=!0,addEventListener("message",function(e){var t=!1,r=e.data.replace(/^(inc\d+)=/,function(e,r){return t=r,""});if(t){var n=document.getElementById(t);n.outerHTML=r+"\n",--pe,pe<=0&&(de?j():setTimeout(ie,0))}})),++pe;var r="inc"+ ++ue;return'<iframe src="'+t+"?id="+r+"&p="+encodeURIComponent(se)+'"id="'+r+'"style="'+le+'" content="text/html;charset=UTF-8"></iframe>'}),ge?document.body.innerHTML=re:de?j():setTimeout(ie,0)}}();


// DisplayMarkdeepOutput.js

BodyHTML=document.body.innerHTML;
BeginTag="<!-- MARKDEEP_BEGIN -->";
EndTag="<!-- MARKDEEP_END -->";
BodyHTML=BodyHTML.slice(BodyHTML.indexOf(BeginTag)+BeginTag.length,BodyHTML.lastIndexOf(EndTag));
document.getElementById("BodyDisplayBox").textContent=BodyHTML;
document.head.innerHTML=FullDocumentHead;


// InvokeMathJax.js

var MathjaxScript=document.createElement("script");
MathjaxScript.type="text/javascript";
MathjaxScript.src="https://cdn.mathjax.org/mathjax/latest/MathJax.js?config=TeX-AMS-MML_HTMLorMML";
document.head.appendChild(MathjaxScript);
