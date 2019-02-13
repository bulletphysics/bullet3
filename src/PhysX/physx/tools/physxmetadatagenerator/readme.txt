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
## Copyright (c) 2018 NVIDIA Corporation. All rights reserved.

If the Physx API is changed, the API level serialization meta data files need to be updated.
This can be done by running:

Windows: generateMetaData.bat 
It looks for python in
	1. environment variable PYTHON
	2. p4sw location (NVidia only)
	3. PATH
	
Linux/MacOS: generateMetaData.sh
	
Linux and osx distributions mostly come with Python pre-installed. On windows please download
and install the latest python version from here: 
	https://www.python.org/downloads 
and make sure to add python to your windows PATH (option in python installer)

The generateMetaData.py tests for meta data files being writable. If not, p4 commands are 
used to open the files for edit before the update. If p4 is not available (or not configured for cmd line usage, see https://www.perforce.com/perforce/r15.1/manuals/p4guide/chapter.configuration.html: "Using config files"), 
the files need to be made writable manually.

Requirements:
Windows: Visual Studio 2015

Testing:
generateMetaData.py -test runs the meta data generation in test mode. This mode checks that the current
set of meta data files is still in sync with the PhysX API (i.e. already generated meta data files). 

