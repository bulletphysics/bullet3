function convertFile(filenameIn, filenameOut, stringname)
	print("\nfilenameOut = " .. filenameOut)
		local f = assert(io.open(filenameIn, "rb"))
		
		local fw = io.open(filenameOut,"w")
		fw:write(string.format("char %s[]={", stringname))
    local block = 10
    while true do
      local bytes = f:read(block)
      if not bytes then break end
      for b in string.gfind(bytes, ".") do
        fw:write(string.format("%u,", string.byte(b)))
      end
      --io.write(string.rep("   ", block - string.len(bytes) + 1))
      --io.write(string.gsub(bytes, "%c", "."), "\n")
      fw:write(string.format("\n"))
    end
    fw:write(string.format("\n};"))
	
end


 newoption {
    trigger     = "binfile",
    value				=	"binpath",
    description = "full path to the binary input file"
  }
  
   newoption {
    trigger     = "cppfile",
    value				=	"path",
    description = "full path to the cpp output file"
  }
  
  newoption {
    trigger     = "stringname",
    value				=	"var",
    description = "name of the variable name in the cppfile that contains the binary data"
  }
  
   newaction {
   trigger     = "bin2cpp",
   description = "convert binary file into cpp source",
   execute = function ()
    convertFile( _OPTIONS["binfile"] , _OPTIONS["cppfile"], _OPTIONS["stringname"])    
 
   end
}