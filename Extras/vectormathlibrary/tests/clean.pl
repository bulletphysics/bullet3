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

$lineno = 0;

sub getLine
{
   local( $line );
   
   $line = <STDIN>;

   while( $line =~ m/^lv2\([^\)]*\)\:$/ )
   {
       $line = <STDIN>;
   }

   $line =~ s/^lv2\([^\)]*\)\: //;

   return $line;
}

while(($line = <STDIN>) !~ m/__begin__/)
{
}

$countSlotLines = 0;

while( $line = &getLine )
{
   $lineno++;
   
   if ( $line =~ m/__end__/ )
   {
      exit;
   }
   
   # if soa print, only save first slot

   if ( $line =~ m/^slot ([1-3])/ )
   {
      while ( $line =~ m/^slot [1-3]/ )
      {
         # skip all lines for this slot

         for ( $i = 0; $i < $slotLines; $i++ )
         {
            $line = &getLine;
         }

         # get next line

         $line = &getLine;
      }

      # stop counting slot lines

      $countSlotLines = 0;
   }
   elsif ( $countSlotLines )
   {
      $slotLines++;
   }

   if ( $line =~ m/^slot 0\:(.?)/ )
   {
      $countSlotLines = 1;

      if ( $1 eq ' ' )
      {
         $line =~ s/^slot 0\: //;
         $slotLines = 0;
      }
      else
      {
         $line = &getLine;
         $slotLines = 1;
      }
   }

   print $line;
}
