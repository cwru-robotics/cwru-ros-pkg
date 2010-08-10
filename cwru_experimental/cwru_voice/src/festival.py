import os

BIN="/usr/bin/festival"

class Festival(object):
    def __init__(self):
        self.p = os.popen("%s --pipe" % BIN, "w")

    def eval(self, scm):
        self.p.write(scm + "\n")
        self.p.flush()

    def say(self, text):
        text = text.replace('"', '')
        self.eval('(SayText "%s")' % str(text))
