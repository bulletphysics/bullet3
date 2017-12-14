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
