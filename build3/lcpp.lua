----------------------------------------------------------------------------
--## lcpp - a C-PreProcessor in Lua 5.1 for LuaJIT ffi
-- 
-- Copyright (C) 2012-2013 Michael Schmoock <michael@willigens.de>
--
--### Links
-- * GitHub page:   [http://github.com/willsteel/lcpp](http://github.com/willsteel/lcpp)
-- * Project page:  [http://lcpp.schmoock.net](http://lcpp.schmoock.net)
-- * Lua:           [http://www.lua.org](http://www.lua.org)
-- * LuaJIT:        [http://luajit.org](http://luajit.org)
-- * Sponsored by:  [http://mmbbq.org](http://mmbbq.org)
-- 
-- It can be used to pre-process LuaJIT ffi C header file input. 
-- It can also be used to preprocess any other code (i.e. Lua itself)
--
-- 	git clone https://github.com/willsteel/lcpp.git
----------------------------------------------------------------------------
--## USAGE
--	-- load lcpp
--	local lcpp = require("lcpp")
--
--	-- use LuaJIT ffi and lcpp to parse cpp code
--	ffi.cdef("#include <your_header.h>")
--
--	-- compile some input
--	local out = lcpp.compile([[
--		#include "myheader.h"
--		#define MAXPATH 260
--		typedef struct somestruct_t {
--			void*          base;
--			size_t         size;
--			wchar_t        path[MAXPATH];
--		} t_exe;
--	]])
--
--	-- the result should be
--	out = [[
--		// <preprocessed content of file "myheader.h">
--		typedef struct somestruct_t {
--			void*          base;
--			size_t         size;
--			wchar_t        path[260];
--		} t_exe;
--	]]
--
--## This CPPs BNF:
--	RULES:
--	CODE              := {LINE}
--	LINE              := {STUFF NEWML} STUFF  NEWL
--	STUFF             := DIRECTIVE | IGNORED_CONTENT
--	DIRECTIVE         := OPTSPACES CMD OPTSPACES DIRECTIVE_NAME WHITESPACES DIRECTIVE_CONTENT WHITESPACES NEWL
--
--	LEAVES:
--	NEWL              := "\n"
--	NEWL_ESC          := "\\n"
--	WHITESPACES       := "[ \t]+"
--	OPTSPACES         := "[ \t]*"
--	COMMENT           := "//(.-)$"
--	MLCOMMENT         := "/[*](.-)[*]/"
--	IGNORED_CONTENT   := "[^#].*"
--	CMD               := "#"
--	DIRECTIVE_NAME    := "include"|"define"|"undef"|"if"|"else"|"elif"|"else if"|"endif"|"ifdef"|"ifndef"|"pragma"|"version"
--	DIRECTIVE_CONTENT := ".*?"
--
--## TODOs:
--	- lcpp.LCPP_LUA for: load, loadfile
--	- "#" operator for stringification
--	- literal concatenation: "foo" "bar" -> "foobar"
--
--## License (MIT)
-- -----------------------------------------------------------------------------
-- Permission is hereby granted, free of charge, to any person obtaining a copy
-- of this software and associated documentation files (the "Software"), to deal
-- in the Software without restriction, including without limitation the rights
-- to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
-- copies of the Software, and to permit persons to whom the Software is
-- furnished to do so, subject to the following conditions:
-- 
-- The above copyright notice and this permission notice shall be included in
-- all copies or substantial portions of the Software.
-- 
-- THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
-- IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
-- FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT.  IN NO EVENT SHALL THE
-- AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
-- LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
-- OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN
-- THE SOFTWARE.
-- 
-- MIT license: http://www.opensource.org/licenses/mit-license.php
-- -----------------------------------------------------------------------------
--
-- @module lcpp
local lcpp = {}

-- CONFIG
lcpp.LCPP_LUA         = false   -- whether to use lcpp to preprocess Lua code (load, loadfile, loadstring...)
lcpp.LCPP_FFI         = true    -- whether to use lcpp as LuaJIT ffi PreProcessor (if used in luaJIT)
lcpp.LCPP_TEST        = false   -- whether to run lcpp unit tests when loading lcpp module
lcpp.ENV              = {}      -- static predefines (env-like)
lcpp.FAST             = true		-- perf. tweaks when enabled. con: breaks minor stuff like __LINE__ macros
lcpp.DEBUG            = false

-- PREDEFINES
local __FILE__        = "__FILE__"
local __LINE__        = "__LINE__"
local __DATE__        = "__DATE__"
local __TIME__        = "__TIME__"
local __LCPP_INDENT__ = "__LCPP_INDENT__"
local __LCPP_INSIDE_HEADERFILE__ = "__LCPP_INSIDE_HEADERFILE__"

-- BNF LEAVES
local ENDL            = "$"
local STARTL          = "^"
local NEWL            = "\n"
local NEWL_BYTE       = NEWL:byte(1)
local NEWL_ESC        = "\\"
local NEWML           = "\\\n"
local CMD             = "#"
local CMD_BYTE        = CMD:byte(1)
local COMMENT         = "^(.-)//.-$"
local MLCOMMENT       = "/[*].-[*]/"
local WHITESPACES     = "%s+"
local OPTSPACES       = "%s*"
local IDENTIFIER      = "[_%a][_%w]*"
local NOIDENTIFIER    = "[^%w_]+"
local FILENAME        = "[0-9a-zA-Z.-_/\\]+"
local TEXT            = ".+"

-- BNF WORDS
local _INCLUDE        = "include"
local _DEFINE         = "define"
local _IFDEF          = "ifdef"
local _IFNDEF         = "ifndef"
local _ENDIF          = "endif"
local _UNDEF          = "undef"
local _IF             = "if"
local _ELSE           = "else"
local _ELIF           = "elif"
local _NOT            = "!"
local _ERROR          = "error"
local _PRAGMA         = "pragma"
local _VERSION				= "version"

-- BNF RULES
local INCLUDE         = STARTL.._INCLUDE..WHITESPACES.."[\"<]("..FILENAME..")[\">]"..OPTSPACES..ENDL
local DEFINE          = STARTL.._DEFINE
local IFDEF           = STARTL.._IFDEF..WHITESPACES.."("..IDENTIFIER..")"..OPTSPACES..ENDL
local IFNDEF          = STARTL.._IFNDEF..WHITESPACES.."("..IDENTIFIER..")"..OPTSPACES..ENDL
local ENDIF           = STARTL.._ENDIF..OPTSPACES.."(.*)"..ENDL
local UNDEF           = STARTL.._UNDEF..WHITESPACES.."("..IDENTIFIER..")"..OPTSPACES..ENDL
local IF              = STARTL.._IF..WHITESPACES.."(.*)"..ENDL
local ELSE            = STARTL.._ELSE..OPTSPACES.."(.*)"..ENDL
local ELIF            = STARTL.._ELIF..WHITESPACES.."(.*)"..ENDL
local ELSEIF          = STARTL.._ELSE..WHITESPACES.._IF..WHITESPACES.."(.*)"..ENDL
local ERROR           = STARTL.._ERROR..WHITESPACES.."("..TEXT..")"..OPTSPACES..ENDL
local ERROR_NOTEXT    = STARTL.._ERROR..OPTSPACES..ENDL	--> not required when we have POSIX regex
local PRAGMA          = STARTL.._PRAGMA
local VERSION         = STARTL.._VERSION



-- speedups
local TRUEMACRO = STARTL.."("..IDENTIFIER..")%s*$"
local REPLMACRO = STARTL.."("..IDENTIFIER..")"..WHITESPACES.."(.+)$"
local FUNCMACRO = STARTL.."("..IDENTIFIER..")%s*%(([%s%w,]*)%)%s*(.*)"


-- ------------
-- LOCAL UTILS
-- ------------
lcpp.STATE = {lineno = 0} -- current state for debugging the last operation
local function error(msg) _G.print(debug.traceback()); _G.error(string.format("lcpp ERR [%04i] %s", lcpp.STATE.lineno, msg)) end
local function print(msg) _G.print(string.format("lcpp INF [%04i] %s", lcpp.STATE.lineno, msg)) end

-- splits a string using a pattern into a table of substrings
local function gsplit(str, pat)
	local function _split(str, pat)
		local t = {}  -- NOTE: use {n = 0} in Lua-5.0
		local fpat = "(.-)"..pat
		local last_end = 1
		local s, e, cap = str:find(fpat, 1)
		while s do
			if s ~= 1 or cap ~= "" then
				coroutine.yield(cap)
			end
			last_end = e + 1
			s, e, cap = str:find(fpat, last_end)
		end
		if last_end <= #str then
			cap = str:sub(last_end)
			coroutine.yield(cap)
		end
	end
	return coroutine.wrap(function() _split(str, pat) end)
end
local function split(str, pat)
	local t = {}
	for str in gsplit(str, pat) do table.insert(t, str) end
	return t
end

-- Checks whether a string starts with a given substring
-- offset is optional
local function strsw(str, pat, offset)
	if not str then return false end
	if not offset then offset = 0 end
	return string.sub(str, 1+offset, string.len(pat)+offset) == pat
end

-- Checks whether a string ends with a given substring
local function strew(str, pat)
	if not str then return false end
	return pat=='' or string.sub(str,-string.len(pat)) == pat
end

-- string trim12 from lua wiki
local function trim(str)
	local from = str:match"^%s*()"
	return from > #str and "" or str:match(".*%S", from)
end

-- returns the number of string occurrences
local function findn(input, what)
	local count = 0
	local offset = 0
	while true do 
			_, offset = string.find(input, what, offset+1, true)
			if not offset then return count end
			count = count + 1
	end
end

-- a lightweight and flexible tokenizer
local function _tokenizer(str, setup)
		local defsetup = {
			-- EXAMPLE patterns have to be pretended with "^" for the tokenizer
			["identifier"] = '^[_%a][_%w]*',
			["number"] = '^[%+%-]?%d+[%.]?%d*',
			["ignore"] = '^%s+', 
			["string"] = true,
			["keywords"] = { 
				-- ["NAME"] = '^pattern',
				-- ...
			},
		}
	if not setup then
		setup = defsetup
	end
	setup.identifier = setup.identifier or defsetup.identifier
	setup.number = setup.number or defsetup.number
	setup.ignore = setup.number or defsetup.ignore
	if nil == setup.string then setup.string = true end
	setup.keywords = setup.keywords or {}

	local strlen = #str
	local i = 1
	local i1, i2
	local keyword
	
	local function find(pat)
		i1, i2 = str:find(pat,i)
		return i1 ~= nil
	end
	
	local function cut()
		return str:sub(i, i2)
	end
	
	local function findKeyword()
		for name, pat in pairs(setup.keywords) do
			local result = find(pat)
			if result then
				keyword = name 
				return true 
			end
		end
	end

	while true do
		if i > strlen then return 'eof', nil, strlen, strlen end
		if find(setup.ignore) then	
			coroutine.yield("ignore", cut(), i1, i2)
		elseif findKeyword() then
			coroutine.yield(keyword, cut(), i1, i2)
		elseif find(setup.number) then
			coroutine.yield('number', tonumber(cut()), i1, i2)
		elseif find(setup.identifier) then
			coroutine.yield('identifier', cut(), i1, i2)
		elseif setup.string and (find('^"[^"]*"') or find("^'[^']*'")) then
			-- strip the quotes
			coroutine.yield('string', cut():sub(2,-2), i1, i2)
		else -- any other unknown character
			i1 = i
			i2 = i
			coroutine.yield('unknown', cut(), i1, i2)
		end
		i = i2+1
	end
end
local function tokenizer(str, setup)
	return coroutine.wrap(function() _tokenizer(str, setup) end)
end


-- ------------
-- PARSER
-- ------------

-- hint: LuaJIT ffi does not rely on us to remove the comments, but maybe other usecases
local function removeComments(input)
		input = string.gsub(input, "//.-\n", "\n") -- remove sl comments
		-- remove multiline comments in a way that it does not break __LINE__ macro
		if lcpp.FAST then
			input = string.gsub(input, "/%*.-%*/", "") -- remove ml comments (stupid method)
		else
			local offset = 0
			local output = {}
			local starti, endi, match, lastendi
			while offset do
				starti, endi, match = input:find("/%*(.-)%*/", offset, false)
				if starti then
					lastendi = endi
					local newlineCount = findn(match, "\n")
					local newlines = string.rep("\n", newlineCount)
					table.insert(output, input:sub(offset+1, starti-1))
					table.insert(output, newlines)
					offset = endi
				else
					offset = nil
					table.insert(output, input:sub((lastendi or 0) + 1))
				end
			end
			input = table.concat(output)
			--error(input)
		end

		return input
end

-- screener: revmoce comments, trim, ml concat...
-- it only splits to cpp input lines and removes comments. it does not tokenize. 
local function screener(input)
	local function _screener(input)
		--input = removeComments(input)

		-- concat mulit-line input.
		local count = 1
		while count > 0 do input, count = string.gsub(input, "^(.-)\\\n(.-)$", "%1 %2\n") end

		-- trim and join blocks not starting with "#"
		local buffer = {}
		for line in gsplit(input, NEWL) do
			--line = trim(line)
			if #line > 0 then
				if line:byte(1) == CMD_BYTE then 
					--line = line:gsub("#%s*(.*)", "#%1")	-- remove optinal whitespaces after "#". reduce triming later.
					if #buffer > 0 then 
						coroutine.yield(table.concat(buffer, NEWL))
						buffer = {} 
					end
					coroutine.yield(line) 
				else
					if lcpp.FAST then
						table.insert(buffer, line) 
					else
						coroutine.yield(line) 
					end
				end
			elseif not lcpp.FAST then
				coroutine.yield(line) 
			end
		end
		if #buffer > 0 then 
			coroutine.yield(table.concat(buffer, NEWL))
		end
	end
	
	return coroutine.wrap(function() _screener(input) end)
end

-- apply currently known macros to input (and returns it)
local function apply(state, input)
	local out = {}
	local functions = {}

	for k, v, start, end_ in tokenizer(input) do
		if k == "identifier" then 
			local repl = v
			local macro = state.defines[v] 
			if macro then
				if type(macro)     == "boolean" then
					repl = ""
				elseif type(macro) == "string" then
					repl = macro
				elseif type(macro) == "number" then
					repl = tostring(macro)
				elseif type(macro) == "function" then
					table.insert(functions, macro)	-- we apply functions in a later step
				end
			end
			table.insert(out, repl)
		else
			table.insert(out, input:sub(start, end_))
		end
	end
	input = table.concat(out)
	for _, func in pairs(functions) do	-- TODO: looks sucky (but works quite nice)
		input = func(input)
	end

	return input
end

-- processes an input line. called from lcpp doWork loop
local function processLine(state, line)
	if not line or #line == 0 then return line end
	local cmd = nil 
	if line:byte(1) == CMD_BYTE then cmd = line:sub(2) end
	--print("processLine(): "..line)


	--[[ SKIPPING ]]-- 
	if state:skip() then return end
	

	--[[ READ NEW DIRECTIVES ]]--
	if cmd then
		-- handle #include ...
		local filename = cmd:match(INCLUDE)
		if filename then
			print("processing header  " .. filename)
			return state:includeFile(filename)
		end
	
		return line
	end

	
	--[[ APPLY MACROS ]]--
	--line = state:apply(line);
	
	return line
end


local function processLine2(state, line)
	if not line or #line == 0 then return line end
	local cmd = nil 
	if line:byte(1) == CMD_BYTE then cmd = line:sub(2) end
	--print("processLine(): "..line)

	if state:defined(__LCPP_INSIDE_HEADERFILE__) then
		--[[ IF/THEN/ELSE STRUCTURAL BLOCKS ]]--
		if cmd then
			local ifdef   = cmd:match(IFDEF)
			local ifexp   = cmd:match(IF)
			local ifndef  = cmd:match(IFNDEF)
			local elif    = cmd:match(ELIF)
			local elseif_ = cmd:match(ELSEIF)
			local else_   = cmd:match(ELSE)
			local endif   = cmd:match(ENDIF)
			local struct  = ifdef or ifexp or ifndef or elif or elseif_ or else_ or endif
			
			if struct then 
				if ifdef   then state:openBlock(state:defined(ifdef))      end
				if ifexp   then state:openBlock(state:parseExpr(ifexp))    end
				if ifndef  then state:openBlock(not state:defined(ifndef)) end
				if elif    then state:elseBlock(state:parseExpr(elif))     end
				if elseif_ then state:elseBlock(state:parseExpr(elseif_))  end
				if else_   then state:elseBlock(true)                      end
				if endif   then state:closeBlock()                         end
				return line
			end
		end
	end


	--[[ SKIPPING ]]-- 
	if state:skip() then return end
	

	--[[ READ NEW DIRECTIVES ]]--
	if cmd then
		-- handle #include ...
		local filename = cmd:match(INCLUDE)
		if filename then
			print("processing header  " .. filename)
			return state:includeFile(filename)
		end
	
		if state:defined(__LCPP_INSIDE_HEADERFILE__) then
			-- handle #undef ...
			local key = cmd:match(UNDEF)
			if type(key) == "string" then
				state:undefine(key)
				return
			end
			
			-- read "#define >FooBar...<" directives
			if cmd:match(DEFINE) then
				local define = trim(cmd:sub(DEFINE:len()+1))
				local macroname, replacement
				
				-- simple "true" defines
				macroname = define:match(TRUEMACRO)
				if macroname then
					state:define(macroname, true)
				end
		
				-- replace macro defines
				macroname, replacement = define:match(REPLMACRO)
				if macroname and replacement then
					state:define(macroname, replacement)
				end
		
				-- read functional macros
				macroname, replacement = state:parseFunction(define)
				if macroname and replacement then
					state:define(macroname, replacement)
				end
				
				return line
			end
			
			-- ignore, because we dont have any pragma directives yet
			if cmd:match(PRAGMA) then 
				return line
			end
			
			-- abort on unknown keywords
			error("unknown directive: "..line)
		else
			return line
		end
	end

	
	--[[ APPLY MACROS ]]--
	--line = state:apply(line);
	
	return line
end


local function doWork(state)
	local function _doWork(state)
		if not state:defined(__FILE__) then state:define(__FILE__, "<USER_CHUNK>", true) end
		local oldIndent = state:getIndent()
		while true do
			local input = state:getLine()
			if not input then break end
			local output = processLine(state, input)
			if not lcpp.FAST and not output then 
				output = "" end -- output empty skipped lines
				
			if lcpp.DEBUG then output = output.." -- "..input end -- input as comment when DEBUG
			
			if output then coroutine.yield(output) end
		end
		if (oldIndent ~= state:getIndent()) then error("indentation level must be balanced within a file. was:"..oldIndent.." is:"..state:getIndent()) end
	end
	return coroutine.wrap(function() _doWork(state) end)
end

local function doWork2(state)
	local function _doWork2(state)
		if not state:defined(__FILE__) then state:define(__FILE__, "<USER_CHUNK>", true) end
		local oldIndent = state:getIndent()
		while true do
			local input = state:getLine()
			if not input then break end
			local output = processLine2(state, input)
			if not lcpp.FAST and not output then output = "" end -- output empty skipped lines
			if lcpp.DEBUG then output = output.." -- "..input end -- input as comment when DEBUG
			if output then coroutine.yield(output) end
		end
		if (oldIndent ~= state:getIndent()) then error("indentation level must be balanced within a file. was:"..oldIndent.." is:"..state:getIndent()) end
	end
	return coroutine.wrap(function() _doWork2(state) end)
end

local function includeFile(state, filename)
	local result, result_state = lcpp.compileHeaderFile("../src/" .. filename, state.defines)
	-- now, we take the define table of the sub file for further processing
	state.defines = result_state.defines
	-- and return the compiled result	
	return result
end

-- sets a global define
local function define(state, key, value, override)
	--print("define:"..key.." type:"..type(value))
	--if value and not override and state:defined(key) then error("already defined: "..key) end
	value = state:prepareMacro(value)
	state.defines[key] = value
end

-- parses CPP exressions
-- i.e.: #if !defined(_UNICODE) && !defined(UNICODE)
--
--BNF:
--  EXPR     -> (BRACKET_OPEN)(EXPR)(BRACKET_CLOSE)
--  EXPR     -> (EXPR)(OR)(EXPR)
--  EXPR     -> (EXPR)(AND)(EXPR)
--  EXPR     -> (NOT)(EXPR)
--  EXPR     -> (FUNCTION)
--  FUNCTION -> (IDENTIFIER)(BRACKET_OPEN)(ARGS)(BRACKET_CLOSE)
--  ARGS     -> ((IDENTIFIER)[(COMMA)(IDENTIFIER)])?
--LEAVES:
--  IGNORE -> " \t"
--  BRACKET_OPEN  -> "("
--  BRACKET_CLOSE -> ")"
--  OR -> "||"
--  AND -> "&&"
--  NOT -> "!"
--  IDENTIFIER -> "[0-9a-zA-Z_]"
--

local LCPP_TOKENIZE_MACRO = {
	string = true,
	keywords = { 
		CONCAT = "^##",
	},
}
local LCPP_TOKENIZE_EXPR = {
	string = false,
	keywords = { 
		NOT = '^!', 
		DEFINED = '^defined', 
		BROPEN = '^[(]', 
		BRCLOSE = '^[)]', 
		AND = '^&&', 
		OR = '^||',
	},
}

local function parseDefined(state, input)
	local result = false
	local bropen = false
	local brclose = false
	local ident = nil
	
	for key, value in input do
		if key == "BROPEN" then
			bropen = true
		end
		if key == "identifier" then
			 ident = value
			 if not bropen then break end
		end
		if key == "BRCLOSE" and ident then
			brclose = true
			break
		end
	end
	
	-- wiht and w/o brackets allowed
	if ident and ((bropen and brclose) or (not bropen and not brclose)) then
		return state:defined(ident)
	end
	
	error("expression parse error: defined(ident)")
end

local function parseExpr(state, input) 
	-- first call gets string input. rest uses tokenizer
	if type(input) == "string" then input = tokenizer(input, LCPP_TOKENIZE_EXPR) end
	local result = false
	local _not = false
	
	for type, value in input do
--		print("type:"..type.." value:"..value)
		if type == "NOT" then
			_not = true
		end
		if type == "BROPEN" then
			return state:parseExpr(input)
		end
		if type == "BRCLOSE" then
			return result
		end
		if type == "AND" then
			return result and state:parseExpr(input)
		end
		if type == "OR" then
			return result or state:parseExpr(input)
		end
		
		if type == "DEFINED" then
			if _not then
				result = not parseDefined(state, input) 
			else
				result = parseDefined(state, input) 
			end
		end
	end
	
	return result
end

-- apply macros chaining and string ops "##" and "#"
local function prepareMacro(state, input)
	if type(input) ~= "string" then return input end
	input = state:apply(input)
	local out = {}
	for k, v, start, end_ in tokenizer(input, LCPP_TOKENIZE_MACRO) do
		if k == "CONCAT" then
			-- remove concat op "##"
		else
			table.insert(out, input:sub(start, end_))
		end
	end
	return table.concat(out)
end

-- i.e.: "MAX(x, y) (((x) > (y)) ? (x) : (y))"
local function parseFunction(state, input)
	if not input then return end
	local name, argsstr, repl = input:match(FUNCMACRO)
	if not name or not argsstr or not repl then return end
	repl = state:prepareMacro(repl)

	-- rename args to %1,%2... for later gsub
	local noargs = 0
	for argname in argsstr:gmatch(IDENTIFIER) do
		noargs = noargs + 1
		repl = repl:gsub(argname, "%%"..noargs)
	end
	
	-- build pattern string:  name(arg, arg, ...)
	local pattern
	if     noargs == 0 then pattern = name.."%s*%(%s*%)"                             -- quick 0 arg version
	elseif noargs == 1 then pattern = name.."%s*%(%s*([^,%)]*)%s*%)"                 -- quick 1 arg version
	elseif noargs == 2 then pattern = name.."%s*%(%s*([^,%)]*)%s*,%s*([^,%)]*)%s*%)" -- quick 2 arg version
	else -- arbitrary arg version
		local buf = {}
		table.insert(buf, name)
		table.insert(buf, "%s*%(%s*")
		for i = 1, noargs do
			table.insert(buf, "([^,%)]*)%s*")
			if i < noargs then
				table.insert(buf, ",%s*")
			end
		end
		table.insert(buf, "%)")
		pattern = table.concat(buf)
	end
	
	-- build macro funcion
	local func = function(input)
		return input:gsub(pattern, repl)
	end
	
	return name, func
end


-- ------------
-- LCPP INTERFACE
-- ------------

--- initialies a lcpp state. not needed manually. handy for testing
function lcpp.init(input, predefines)
	-- create sate var
	local state     = {}              -- init the state object
	state.defines   = {}              -- the table of known defines and replacements
	state.screener  = screener(input)
	state.lineno    = 0               -- the current line number
	state.stack     = {}              -- stores wether the current stack level is to be included
	state.once      = {}              -- stack level was once true (first if that evals to true)
	
	-- funcs
	state.define = define
	state.undefine = function(state, key)
		state:define(key, nil)
	end
	state.defined = function(state, key)
		return state.defines[key] ~= nil
	end
	state.apply = apply
	state.includeFile = includeFile
	state.doWork = doWork
	state.doWork2 = doWork2
	state.getIndent = function(state)
		return #state.stack
	end
	state.openBlock = function(state, bool)
		state.stack[#state.stack+1] = bool
		state.once [#state.once+1]  = bool
		state:define(__LCPP_INDENT__, state:getIndent(), true)
	end
	state.elseBlock = function(state, bool)
		if state.once[#state.once] then
			state.stack[#state.stack] = false
		else
			state.stack[#state.stack] = bool
			if bool then state.once[#state.once] = true end
		end
	end
	state.closeBlock = function(state)
		state.stack[#state.stack] = nil
		state.once [#state.once]  = nil
		state:define(__LCPP_INDENT__, state:getIndent(), true)
		if state:getIndent() < 0 then error("Unopened block detected. Indentaion problem.") end
	end
	state.skip = function(state)
		for i = 1, #state.stack do
			if not state.stack[i] then return true end
		end
		return false
	end
	state.getLine = function(state)
		state.lineno = state.lineno + 1
		state:define(__LINE__, state.lineno, true)
		return state.screener()
	end
	state.prepareMacro = prepareMacro
	state.parseExpr = parseExpr
	state.parseFunction = parseFunction
	
	-- predefines
	state:define(__DATE__, os.date("%B %d %Y"), true)
	state:define(__TIME__, os.date("%H:%M:%S"), true)
	state:define(__LINE__, state.lineno, true)
	state:define(__LCPP_INDENT__, state:getIndent(), true)
	predefines = predefines or {}
	for k,v in pairs(lcpp.ENV) do	state:define(k, v, true) end	-- static ones
	for k,v in pairs(predefines) do	state:define(k, v, true) end
	
	if lcpp.LCPP_TEST then lcpp.STATE = state end -- activate static state debugging

	return state
end

--- the preprocessors main function.
-- returns the preprocessed output as a string.
-- @param code data as string
-- @param predefines OPTIONAL a table of predefined variables
-- @usage lcpp.compile("#define bar 0x1337\nstatic const int foo = bar;")
-- @usage lcpp.compile("#define bar 0x1337\nstatic const int foo = bar;", {["bar"] = "0x1338"})
function lcpp.compile(code, predefines)
	local state = lcpp.init(code, predefines)
	local buf = {}
	for output in state:doWork() do
		table.insert(buf, output)
	end
	local output = table.concat(buf, NEWL)
	if lcpp.DEBUG then print(output) end
	return output, state
end

function lcpp.compile2(code, predefines)
	local state = lcpp.init(code, predefines)
	state:define(__LCPP_INSIDE_HEADERFILE__,true)
	local buf = {}
	for output in state:doWork2() do
		table.insert(buf, output)
	end
	local output = table.concat(buf, NEWL)
	if lcpp.DEBUG then print(output) end
	return output, state
end

--- preprocesses a file
-- @param filename the file to read
-- @param predefines OPTIONAL a table of predefined variables
-- @usage out, state = lcpp.compileFile("../odbg/plugin.h", {["MAXPATH"]=260, ["UNICODE"]=true})
function lcpp.compileFile(filename, predefines)
	if not filename then error("processFile() arg1 has to be a string") end
	local file = io.open(filename, 'r')
	if not file then error("file not found: "..filename) end
	local code = file:read('*a')
	predefines = predefines or {}
	predefines[__FILE__] = filename
	return lcpp.compile(code, predefines)
end

function lcpp.compileHeaderFile(filename, predefines)
	if not filename then error("processFile() arg1 has to be a string") end
	local file = io.open(filename, 'r')
	if not file then error("file not found: "..filename) end
	local code = file:read('*a')
	predefines = predefines or {}
	predefines[__FILE__] = filename
	return lcpp.compile2(code, predefines)
end



-- ------------
-- REGISTER LCPP
-- ------------

--- disable lcpp processing for ffi, loadstring and such
lcpp.disable = function()
	if lcpp.LCPP_LUA then
		-- activate LCPP_LUA actually does anything useful
		-- _G.loadstring = _G.loadstring_lcpp_backup
	end	
	
	if lcpp.LCPP_FFI and pcall(require, "ffi") then
		ffi = require("ffi")
		if ffi.lcpp_cdef_backup then
			ffi.cdef = ffi.lcpp_cdef_backup
			ffi.lcpp_cdef_backup = nil
		end
	end
end

--- (re)enable lcpp processing for ffi, loadstring and such
lcpp.enable = function()
	-- Use LCPP to process Lua code (load, loadfile, loadstring...)
	if lcpp.LCPP_LUA then
		-- TODO: make it properly work on all functions
		error("lcpp.LCPP_LUA = true -- not properly implemented yet");
		_G.loadstring_lcpp_backup = _G.loadstring
		_G.loadstring = function(str, chunk) 
			return loadstring_lcpp_backup(lcpp.compile(str), chunk) 
		end
	end
	-- Use LCPP as LuaJIT PreProcessor if used inside LuaJIT. i.e. Hook ffi.cdef
	if lcpp.LCPP_FFI and pcall(require, "ffi") then
		ffi = require("ffi")
		if not ffi.lcpp_cdef_backup then
			if not ffi.lcpp_defs then ffi.lcpp_defs = {} end -- defs are stored and reused
			ffi.lcpp = function(input) 
				local output, state = lcpp.compile(input, ffi.lcpp_defs)
				ffi.lcpp_defs = state.defines
				return output	
			end
			ffi.lcpp_cdef_backup = ffi.cdef
			ffi.cdef = function(input) return ffi.lcpp_cdef_backup(ffi.lcpp(input)) end
		end
	end
end

lcpp.enable()
return lcpp