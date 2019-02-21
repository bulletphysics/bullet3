##
## Redistribution and use in source and binary forms, with or without
## modification, are permitted provided that the following conditions
## are met:
##  * Redistributions of source code must retain the above copyright
##    notice, this list of conditions and the following disclaimer.
##  * Redistributions in binary form must reproduce the above copyright
##    notice, this list of conditions and the following disclaimer in the
##    documentation and/or other materials provided with the distribution.
##  * Neither the name of NVIDIA CORPORATION nor the names of its
##    contributors may be used to endorse or promote products derived
##    from this software without specific prior written permission.
##
## THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS ``AS IS'' AND ANY
## EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
## IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR
## PURPOSE ARE DISCLAIMED.  IN NO EVENT SHALL THE COPYRIGHT OWNER OR
## CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL,
## EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO,
## PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR
## PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY
## OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
## (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
## OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
##
## Copyright (c) 2016-2019 NVIDIA Corporation. All rights reserved.

# general utility module
from __future__ import print_function
from __future__ import with_statement

import os
import sys
import re
from . import utils

def compareMetaDataDirectories(candidateDir, referenceDir):

	print("reference dir:", referenceDir)
	print("candidate dir:", candidateDir)

	if not _checkFileExistence(candidateDir, referenceDir):
		return False
	
	referenceFiles = utils.list_autogen_files(referenceDir)
	
	#get corresponding candidate files without relying on os.walk order
	def mapRefToCand(refFile):
		return os.path.join(candidateDir, os.path.relpath(refFile, referenceDir))
	candidateFiles = [mapRefToCand(f) for f in referenceFiles]
	
	for (fileCand, fileRef) in zip(candidateFiles, referenceFiles):
		
		timeCand = os.path.getmtime(fileCand)
		timeRef = os.path.getmtime(fileRef)
		
		if timeCand <= timeRef:
			print("last modified time of candidate is not later than last modified time of reference:")
			print("candidate:", fileCand, "\n", "reference:", fileRef)
			print("ref:", timeRef)
			print("cand:", timeCand)
			return False
		
		#_read_file_content will remove line endings(windows/unix), but not ignore empty lines
		candLines = _read_file_content(fileCand)
		refLines = _read_file_content(fileRef)
		if not (candLines and refLines):
			return False
				
		if len(candLines) != len(refLines):
			print("files got different number of lines:")
			print("candidate:", fileCand, "\n", "reference:", fileRef)
			print("ref:", len(refLines))
			print("cand:", len(candLines))
			return False

		for (i, (lineCand, lineRef)) in enumerate(zip(candLines, refLines)):
			if (lineCand != lineRef):
				print("candidate line is not equal to refence line:")
				print("candidate:", fileCand, "\n", "reference:", fileRef)
				print("@line number:", i)
				print("ref:", lineRef)
				print("cand:", lineCand)
				return False
			
	return True

##############################################################################
# internal functions
##############################################################################
	
#will remove line endings(windows/unix), but not ignore empty lines
def _read_file_content(filePath):
	lines = []
	try:
		with open(filePath, "r") as file:
			for line in file:
				lines.append(line.rstrip())
	except:
		print("issue with reading file:", filePath)
	
	return lines
	
def _checkFileExistence(candidateDir, referenceDir):
	candidateSet = set([os.path.relpath(f, candidateDir) for f in utils.list_autogen_files(candidateDir)])
	referenceSet = set([os.path.relpath(f, referenceDir) for f in utils.list_autogen_files(referenceDir)])
	
	missingSet = referenceSet - candidateSet
	if missingSet:
		print("the following files are missing from the candidates:\n", "\n".join(missingSet))
		return False
		
	excessSet = candidateSet - referenceSet
	if excessSet:
		print("too many candidate files:\n", "\n".join(excessSet))
		return False
	
	return True


