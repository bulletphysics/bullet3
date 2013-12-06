
This is a modified CDTestFramework to test Bullet's dynamic AABB tree against a SAP. It's the same demo as here: http://bulletphysics.com/ftp/pub/test/physics/demos/CDTestFramework2.70.zip

But I added an extra challenger: http://www.codercorner.com/Code/SweepAndPrune2.rar. This is the code described in this document: http://www.codercorner.com/SAP.pdf

So there are 4 tests:
- OPCODE's "box pruning"
- Bullet's Multi SAP
- Bullet's dbvt (dynamic AABB tree)
- OPCODE's array-based SAP

For 8192 boxes and 10% of them moving, OPCODE's SAP is roughly as fast as dbvt (and twice faster than Bullet's SAP on my machine). For less boxes (say 1024 or 2048), OPCODE's SAP is faster than dbvt. Figures and "winner" vary a lot depending on the number of objects and how many of them are moving each frame.

If you're interested you can see for yourself:
- download Bullet 2.70
- replace the content of this directory with the new files: \bullet-2.70\bullet-2.70\Extras\CDTestFramework


Cheers,

- Pierre Terdiman
August 31, 2008
