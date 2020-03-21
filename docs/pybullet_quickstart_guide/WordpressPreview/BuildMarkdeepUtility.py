import re

if (__name__ == "__main__"):
  # Assemble the script which embeds the Markdeep page into the preview blog
  PreviewBlogPage = open("PreviewBlogPage.htm", "rb").read().decode("utf-8")
  HeadMatch = re.search("<head(.*?)>(.*?)</head>", PreviewBlogPage, re.DOTALL)
  HeadAttributes = HeadMatch.group(1)
  FullDocumentHead = HeadMatch.group(2)
  BodyMatch = re.search("<body(.*?)>(.*?)</body>", PreviewBlogPage, re.DOTALL)
  BodyAttributes = BodyMatch.group(1)
  FullPreviewBody = BodyMatch.group(2)
  ArticleHTMLCodeMacro = "$(ARTICLE_HTML_CODE)"
  iArticleHTMLCodeMacro = FullPreviewBody.find(ArticleHTMLCodeMacro)
  DocumentBodyPrefix = FullPreviewBody[0:iArticleHTMLCodeMacro]
  DocumentBodySuffix = FullPreviewBody[iArticleHTMLCodeMacro + len(ArticleHTMLCodeMacro):]
  FullPrepareHTMLCode = open("PrepareHTML.js", "rb").read().decode("utf-8")
  ReplacementList = [("$(FULL_DOCUMENT_HEAD)", FullDocumentHead),
                     ("$(DOCUMENT_BODY_PREFIX)", DocumentBodyPrefix),
                     ("$(DOCUMENT_BODY_SUFFIX)", DocumentBodySuffix)]
  for Macro, Replacement in ReplacementList:
    FullPrepareHTMLCode = FullPrepareHTMLCode.replace(
        Macro,
        Replacement.replace("\r\n", "\\r\\n\\\r\n").replace("'", "\\'"))
  # Generate code which sets body and head attributes appropriately
  for Element, AttributeCode in [("head", HeadAttributes), ("body", BodyAttributes)]:
    FullPrepareHTMLCode += "\r\n// Setting " + Element + " attributes\r\n"
    for Match in re.finditer("(\\w+)=\\\"(.*?)\\\"", AttributeCode):
      FullPrepareHTMLCode += "document." + Element + ".setAttribute(\"" + Match.group(
          1) + "\",\"" + Match.group(2) + "\");\r\n"
  open("PrepareHTML.full.js", "wb").write(FullPrepareHTMLCode.encode("utf-8"))

  # Concatenate all the scripts together
  SourceFileList = [
      "PrepareHTML.full.js", "SetMarkdeepMode.js", "markdeep.min.js", "DisplayMarkdeepOutput.js",
      "InvokeMathJax.js"
  ]
  OutputCode = "\r\n\r\n".join([
      "// " + SourceFile + "\r\n\r\n" + open(SourceFile, "rb").read().decode("utf-8")
      for SourceFile in SourceFileList
  ])
  OutputFile = open("MarkdeepUtility.js", "wb")
  OutputFile.write(OutputCode.encode("utf-8"))
  OutputFile.close()
  print("Done.")
