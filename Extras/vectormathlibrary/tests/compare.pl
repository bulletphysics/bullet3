#!/usr/bin/perl

#
#  Copyright (C) 2006, 2007 Sony Computer Entertainment Inc.
#  All rights reserved.
#
#  Redistribution and use in source and binary forms,
#  with or without modification, are permitted provided that the
#  following conditions are met:
#   * Redistributions of source code must retain the above copyright
#     notice, this list of conditions and the following disclaimer.
#   * Redistributions in binary form must reproduce the above copyright
#     notice, this list of conditions and the following disclaimer in the
#     documentation and/or other materials provided with the distribution.
#   * Neither the name of the Sony Computer Entertainment Inc nor the names
#     of its contributors may be used to endorse or promote products derived
#     from this software without specific prior written permission.
#
#  THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
#  AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
#  IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
#  ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE
#  LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
#  CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
#  SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
#  INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
#  CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
#  ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
#  POSSIBILITY OF SUCH DAMAGE.
#

$file1 = $ARGV[0];
$file2 = $ARGV[1];

if (!open(FILE1, "<$file1"))
{
    print "Couldn't open $file1\n";
    exit;
}

if (!open(FILE2, "<$file2"))
{
    print "Couldn't open $file2\n";
    exit;
}

print "Comparing $file1 $file2\n";

$lineno1 = 0;
$lineno2 = 0;

while(($line1 = <FILE1>) && ($line2 = <FILE2>))
{
   $lineno1++;
   $lineno2++;
   
   if ( $line1 =~ m/\:$/ )
   {
      $line1 = <FILE1>;
      $lineno1++;
   }
   
   if ( $line2 =~ m/\:$/ )
   {
      $line2 = <FILE2>;
      $lineno2++;
   }

   $line1 =~ s/^.*\: //g;
   $line2 =~ s/^.*\: //g;

   @words1 = split(/ /,$line1);
   @words2 = split(/ /,$line2);
   
   for ($i = 0; $i < @words1; $i++)
   {
      $word1 = $words1[$i];
      $word2 = $words2[$i];
      
      $word1 =~ s/\s//g;
      $word2 =~ s/\s//g;
      
      if ( $word1 ne $word2 )
      {
         $error = abs($word1 - $word2);

         $limit = abs(1e-4 * $word1);
         
         if ( $error > $limit && !( abs($word1) < 1e-4 && $error < 1e-4 ) )
         {
            print "$lineno1: $word1 $lineno2: $word2\n";
         }
      }
   }
}
