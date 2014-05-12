#!/usr/bin/python
#
# Copyright (c) 2011, The Native Client Authors.  All rights reserved.
# Use of this source code is governed by a BSD-style license that can be
# found in the LICENSE file.
#

"""A tiny web server.

This is intended to be used for testing, and only run from within the examples
directory.
"""

import BaseHTTPServer
import logging
import os
import SimpleHTTPServer
import SocketServer
import sys
import urlparse

logging.getLogger().setLevel(logging.INFO)

# Using 'localhost' means that we only accept connections
# via the loop back interface.
SERVER_PORT = 5103
SERVER_HOST = ''

# We only run from the examples directory (the one that contains scons-out), so
# that not too much is exposed via this HTTP server.  Everything in the
# directory is served, so there should never be anything potentially sensitive
# in the serving directory, especially if the machine might be a
# multi-user machine and not all users are trusted.  We only serve via
# the loopback interface.

SAFE_DIR_COMPONENTS = ['bin_html']
SAFE_DIR_SUFFIX = apply(os.path.join, SAFE_DIR_COMPONENTS)

def SanityCheckDirectory():
  if os.getcwd().endswith(SAFE_DIR_SUFFIX):
    return
  logging.error('httpd.py should only be run from the %s', SAFE_DIR_SUFFIX)
  logging.error('directory for testing purposes.')
  logging.error('We are currently in %s', os.getcwd())
  sys.exit(1)


# An HTTP server that will quit when |is_running| is set to False.  We also use
# SocketServer.ThreadingMixIn in order to handle requests asynchronously for
# faster responses.
class QuittableHTTPServer(SocketServer.ThreadingMixIn,
                          BaseHTTPServer.HTTPServer):
  def serve_forever(self, timeout=0.5):
    self.is_running = True
    self.timeout = timeout
    while self.is_running:
      self.handle_request()

  def shutdown(self):
    self.is_running = False
    return 1


# "Safely" split a string at |sep| into a [key, value] pair.  If |sep| does not
# exist in |str|, then the entire |str| is the key and the value is set to an
# empty string.
def KeyValuePair(str, sep='='):
  if sep in str:
    return str.split(sep)
  else:
    return [str, '']


# A small handler that looks for '?quit=1' query in the path and shuts itself
# down if it finds that parameter.
class QuittableHTTPHandler(SimpleHTTPServer.SimpleHTTPRequestHandler):
  def do_GET(self):
    (_, _, _, query, _) = urlparse.urlsplit(self.path)
    url_params = dict([KeyValuePair(key_value)
                      for key_value in query.split('&')])
    if 'quit' in url_params and '1' in url_params['quit']:
      self.send_response(200, 'OK')
      self.send_header('Content-type', 'text/html')
      self.send_header('Content-length', '0')
      self.end_headers()
      self.server.shutdown()
      return

    SimpleHTTPServer.SimpleHTTPRequestHandler.do_GET(self)


def Run(server_address,
        server_class=QuittableHTTPServer,
        handler_class=QuittableHTTPHandler):
  httpd = server_class(server_address, handler_class)
  logging.info("Starting local server on port %d", server_address[1])
  logging.info("To shut down send http://localhost:%d?quit=1",
               server_address[1])
  try:
    httpd.serve_forever()
  except KeyboardInterrupt:
    logging.info("Received keyboard interrupt.")
    httpd.server_close()

  logging.info("Shutting down local server on port %d", server_address[1])


if __name__ == '__main__':
  SanityCheckDirectory()
  if len(sys.argv) > 1:
    Run((SERVER_HOST, int(sys.argv[1])))
  else:
    Run((SERVER_HOST, SERVER_PORT))
  sys.exit(0)
