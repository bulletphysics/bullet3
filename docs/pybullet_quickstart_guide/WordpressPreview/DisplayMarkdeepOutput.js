BodyHTML=document.body.innerHTML;
BeginTag="<!-- MARKDEEP_BEGIN -->";
EndTag="<!-- MARKDEEP_END -->";
BodyHTML=BodyHTML.slice(BodyHTML.indexOf(BeginTag)+BeginTag.length,BodyHTML.lastIndexOf(EndTag));
document.getElementById("BodyDisplayBox").textContent=BodyHTML;
document.head.innerHTML=FullDocumentHead;
