/*
  Bullet for XNA Copyright (c) 2003-2007 Vsevolod Klementjev http://www.codeplex.com/xnadevru
  Bullet original C++ version Copyright (c) 2003-2007 Erwin Coumans http://bulletphysics.com

  This software is provided 'as-is', without any express or implied
  warranty.  In no event will the authors be held liable for any damages
  arising from the use of this software.

  Permission is granted to anyone to use this software for any purpose,
  including commercial applications, and to alter it and redistribute it
  freely, subject to the following restrictions:

  1. The origin of this software must not be misrepresented; you must not
     claim that you wrote the original software. If you use this software
     in a product, an acknowledgment in the product documentation would be
     appreciated but is not required.
  2. Altered source versions must be plainly marked as such, and must not be
     misrepresented as being the original software.
  3. This notice may not be removed or altered from any source distribution.
*/

using System;
using System.Collections.Generic;
using System.Text;
using Microsoft.Xna.Framework;

namespace XnaDevRu.BulletX
{
	/// <summary>
	/// GjkEpaSolver contributed under zlib by Nathanael Presson
	/// </summary>
	public class GjkEpaSolver
	{
		public struct Results
		{
			public enum Status
			{
				Separated,		/* Shapes doesnt penetrate												*/
				Penetrating,	/* Shapes are penetrating												*/
				GjkFailed,		/* GJK phase fail, no big issue, shapes are probably just 'touching'	*/
				EpaFailed,		/* EPA phase fail, bigger problem, need to save parameters, and debug	*/
			}

			private Vector3[] _witnesses;
			private Vector3 _normal;
			private float _depth;
			private int _epaIterations;
			private int _gjkIterations;
			private Status _status;

			public Vector3[] Witnesses { get { return _witnesses; } set { _witnesses = value; } }
			public Vector3 Normal { get { return _normal; } set { _normal = value; } }
			public float Depth { get { return _depth; } set { _depth = value; } }
			public int EpaIterations { get { return _epaIterations; } set { _epaIterations = value; } }
			public int GjkIterations { get { return _gjkIterations; } set { _gjkIterations = value; } }
			public Status ResultStatus { get { return _status; } set { _status = value; } }
		}

		public static bool Collide(ConvexShape shapeA, Matrix wtrsA,
					ConvexShape shapeB, Matrix wtrsB,
					float radialmargin,
					out Results results)
		{
			/* Initialize					*/
			results = new Results();
			results.Witnesses = new Vector3[2];
			results.Witnesses[0] =
			results.Witnesses[1] =
			results.Normal = new Vector3();
			results.Depth = 0;
			results.ResultStatus = Results.Status.Separated;
			results.EpaIterations = 0;
			results.GjkIterations = 0;
			/* Use GJK to locate origin		*/
			GjkEpa.Gjk gjk = new GjkEpa.Gjk(wtrsA, wtrsA.Translation, shapeA,
							wtrsB, wtrsB.Translation, shapeB,
							radialmargin + GjkEpa.EpaAccuracy);
			bool collide = gjk.SearchOrigin();
			results.GjkIterations = gjk.Iterations + 1;
			if (collide)
			{
				/* Then EPA for penetration depth	*/
				GjkEpa.Epa epa = new GjkEpa.Epa(gjk);
				float pd = epa.EvaluatePD();
				results.EpaIterations = epa.Iterations + 1;
				if (pd > 0)
				{
					results.ResultStatus = Results.Status.Penetrating;
					results.Normal = epa.Normal;
					results.Depth = pd;
					results.Witnesses[0] = epa.Nearest[0];
					results.Witnesses[1] = epa.Nearest[1];
					return true;
				}
				else { if (epa.Failed) results.ResultStatus = Results.Status.EpaFailed; }
			}
			else { if (gjk.Failed) results.ResultStatus = Results.Status.GjkFailed; }
			return false;
		}
	}
}
