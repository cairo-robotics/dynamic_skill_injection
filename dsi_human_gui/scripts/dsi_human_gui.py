#!/usr/bin/env python

import sys

from dsi_human_gui.gui import Interface
from rqt_gui.main import Main

plugin = 'dsi_human_gui'
main = Main(filename=plugin)
sys.exit(main.main(standalone=plugin))
