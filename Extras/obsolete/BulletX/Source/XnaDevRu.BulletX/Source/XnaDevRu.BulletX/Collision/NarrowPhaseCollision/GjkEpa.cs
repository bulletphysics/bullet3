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
	/// GJK-EPA collision solver by Nathanael Presson
	/// Nov.2006
	/// </summary>
	public class GjkEpa
	{
		//private static readonly int _precision = 1 /* U(sizeof(F) == 4)*/;

		private static readonly float _infinity = MathHelper.Infinity;
		//private static readonly float _pi = (float)Math.PI;
		private static readonly float _twoPi = (float)(Math.PI * 2);

		private static readonly int _gjkMaxIterations = 128;
		private static readonly int _gjkHashSize = 1 << 6;
		private static readonly int _gjkHashMask = _gjkHashSize - 1;
		private static readonly float _gjkInSimplexEpsilon = 0.0001f;
		private static readonly float _gjkSquareInSimplexEpsilon = _gjkInSimplexEpsilon * _gjkInSimplexEpsilon;

		private static readonly int _epaMaxIterations = 256;
		private static readonly float _epaInFaceEpsilon = 0.01f;
		private static readonly float _epaAccuracy = 0.001f;

		public static float EpaAccuracy { get { return _epaAccuracy; } }

		private static float Abs(float v) { return (v < 0 ? -v : v); }
		private static float Sign(float v) { return (v < 0 ? -1 : 1); }

		static void Swap<T>(ref T a, ref T b)
		{
			T t = a;
			a = b;
			b = t;
		}

		public class Gjk
		{
			public class MinkowskiVertice
			{
				private Vector3 _vertice;	/* Minkowski vertice	*/
				private Vector3 _ray;		/* Ray					*/

				public Vector3 Vertice { get { return _vertice; } set { _vertice = value; } }
				public Vector3 Ray { get { return _ray; } set { _ray = value; } }
			}
			public class He
			{
				private Vector3 _ray;
				private He _next;

				public He Next { get { return _next; } set { _next = value; } }
				public Vector3 Ray { get { return _ray; } set { _ray = value; } }
			}

			private He[] _table = new He[_gjkHashSize];
			private Matrix[] _wrotations = new Matrix[2];
			private Vector3[] _positions = new Vector3[2];
			private ConvexShape[] _shapes = new ConvexShape[2];
			private MinkowskiVertice[] _simplex = new MinkowskiVertice[5];
			private Vector3 _ray;
			private int _order;
			private int _iterations;
			private float _margin;
			private bool _failed;

			public Gjk(Matrix wrotationA, Vector3 positionA, ConvexShape shapeA,
						Matrix wrotationB, Vector3 positionB, ConvexShape shapeB)
				: this(wrotationA, positionA, shapeA, wrotationB, positionB, shapeB, 0) { }

			public Gjk(Matrix wrotationA, Vector3 positionA, ConvexShape shapeA,
						Matrix wrotationB, Vector3 positionB, ConvexShape shapeB,
						float pmargin)
			{
                for (int i = 0; i < _simplex.Length; i++)
                    _simplex[i] = new MinkowskiVertice();

                for (int i = 0; i < _wrotations.Length; i++)
                    _wrotations[i] = new Matrix();

                for (int i = 0; i < _positions.Length; i++)
                    _positions[i] = new Vector3();

                _wrotations[0] = wrotationA; _positions[0] = positionA; 
				_shapes[0] = shapeA;
                _wrotations[0].Translation = Vector3.Zero;
				_wrotations[1] = wrotationB; _positions[1] = positionB; 
				_shapes[1] = shapeB;
                _wrotations[1].Translation = Vector3.Zero;
				//sablock = sa->BeginBlock();
				_margin = pmargin;
				_failed = false;
			}

			public bool Failed { get { return _failed; } }
			public int Iterations { get { return _iterations; } }
			public int Order { get { return _order; } }
			public MinkowskiVertice[] Simplex { get { return _simplex; } }

			public int Hash(Vector3 v)
			{
				int h = ((int)(v.X * 15461) ^ (int)(v.Y * 83003) ^ (int)(v.Z * 15473));
				return (h * 169639) & _gjkHashMask;
			}

			public bool FetchSupport()
			{
				int h = Hash(_ray);
				He e = _table[h];
				while (e != null)
				{
					if (e.Ray == _ray)
					{
						--_order;
						return (false);
					}
					else e = e.Next;
				}
				e = new He(); 
				e.Ray = _ray; 
				e.Next = _table[h]; 
				_table[h] = e;
				Support(_ray, ref _simplex[++_order]);
				return (Vector3.Dot(_ray, _simplex[_order].Vertice) > 0);
			}

			public Vector3 LocalSupport(Vector3 d, int i)
			{
                Matrix m = _wrotations[i];
                m.Translation = Vector3.Zero;
                Vector3 vtx = Vector3.TransformNormal(d, m);
                Vector3 result = MathHelper.MatrixToVector(_wrotations[i], _shapes[i].LocalGetSupportingVertex(vtx));
                return (result + _positions[i]);
			}

			public void Support(Vector3 d, ref MinkowskiVertice v)
			{
				v.Ray = d;
				v.Vertice = LocalSupport(d, 0) - LocalSupport(-d, 1) + d * _margin;
			}

			public bool SolveSimplex2(Vector3 ao, Vector3 ab)
			{
				if (Vector3.Dot(ab, ao) >= 0)
				{
					Vector3 cabo = Vector3.Cross(ab, ao);
					if (cabo.LengthSquared() > _gjkSquareInSimplexEpsilon)
					{ _ray = Vector3.Cross(cabo, ab); }
					else
					{ return true; }
				}
				else
				{
                    _order = 0;
					_simplex[0].Ray = _simplex[1].Ray;
					_simplex[0].Vertice = _simplex[1].Vertice;

                    _ray = ao; 
                }
				return false;
			}

			public bool SolveSimplex3(Vector3 ao, Vector3 ab, Vector3 ac)
			{
				return (SolveSimplex3a(ao, ab, ac, Vector3.Cross(ab, ac)));
			}

			public bool SolveSimplex3a(Vector3 ao, Vector3 ab, Vector3 ac, Vector3 cabc)
			{
				if ((Vector3.Dot(Vector3.Cross(cabc, ab), ao)) < -_gjkInSimplexEpsilon)
				{
                    _order = 1;
					_simplex[0].Vertice = _simplex[1].Vertice;
					_simplex[0].Ray = _simplex[1].Ray;

					_simplex[1].Vertice = _simplex[2].Vertice;
					_simplex[1].Ray = _simplex[2].Ray;

                    return (SolveSimplex2(ao, ab));
                }
				else if (Vector3.Dot(Vector3.Cross(cabc, ac), ao) > +_gjkInSimplexEpsilon)
				{
                    _order = 1;
					_simplex[1].Vertice = _simplex[2].Vertice;
					_simplex[1].Ray = _simplex[2].Ray;

                    return (SolveSimplex2(ao, ac));
                }
				else
				{
					float d = Vector3.Dot(cabc, ao);
					if (Abs(d) > _gjkInSimplexEpsilon)
					{
						if (d > 0)
						{ _ray = cabc; }
						else
						{ _ray = -cabc; Swap<MinkowskiVertice>(ref _simplex[0], ref _simplex[1]); }
						return (false);
					}
					else return (true);
				}
			}

			public bool SolveSimplex4(Vector3 ao, Vector3 ab, Vector3 ac, Vector3 ad)
			{
				Vector3 crs;
				if (Vector3.Dot((crs = Vector3.Cross(ab, ac)), ao) > _gjkInSimplexEpsilon)
				{
					_order = 2;
					_simplex[0].Vertice = _simplex[1].Vertice;
					_simplex[0].Ray = _simplex[1].Ray;

					_simplex[1].Vertice = _simplex[2].Vertice;
					_simplex[1].Ray = _simplex[2].Ray;

					_simplex[2].Vertice = _simplex[3].Vertice;
					_simplex[2].Ray = _simplex[3].Ray;
                    
                    return (SolveSimplex3a(ao, ab, ac, crs));
				}
				else if (Vector3.Dot((crs = Vector3.Cross(ac, ad)), ao) > _gjkInSimplexEpsilon)
				{ 
                    _order = 2;
					_simplex[2].Vertice = _simplex[3].Vertice;
					_simplex[2].Ray = _simplex[3].Ray;
                    
                    return (SolveSimplex3a(ao, ac, ad, crs)); 
                }
				else if (Vector3.Dot((crs = Vector3.Cross(ad, ab)), ao) > _gjkInSimplexEpsilon)
				{
					_order = 2;

					_simplex[1].Vertice = _simplex[0].Vertice;
					_simplex[1].Ray = _simplex[0].Ray;

					_simplex[0].Vertice = _simplex[2].Vertice;
					_simplex[0].Ray = _simplex[2].Ray;

					_simplex[2].Vertice = _simplex[3].Vertice;
					_simplex[2].Ray = _simplex[3].Ray; 
                    
                    return (SolveSimplex3a(ao, ad, ab, crs));
				}
				else return (true);
			}

			public bool SearchOrigin()
			{
				return SearchOrigin(new Vector3(1, 0, 0));
			}

			public bool SearchOrigin(Vector3 initray)
			{
				_iterations = 0;
				unchecked
				{
					_order = (int)(-1);
				}
				_failed = false;
				_ray = Vector3.Normalize(initray);

				//ClearMemory(table, sizeof(void*) * GJK_hashsize);
                for (int i = 0; i < _table.Length; i++)
                    _table[i] = null;
				FetchSupport();
				_ray = -_simplex[0].Vertice;
				for (; _iterations < _gjkMaxIterations; ++_iterations)
				{
					float rl = _ray.Length();
					_ray /= rl > 0 ? rl : 1;
					if (FetchSupport())
					{
						bool found = (false);
						switch (_order)
						{
							case 1: found = SolveSimplex2(-_simplex[1].Vertice, _simplex[0].Vertice - _simplex[1].Vertice); break;
							case 2: found = SolveSimplex3(-_simplex[2].Vertice, _simplex[1].Vertice - _simplex[2].Vertice, _simplex[0].Vertice - _simplex[2].Vertice); break;
							case 3: found = SolveSimplex4(-_simplex[3].Vertice, _simplex[2].Vertice - _simplex[3].Vertice, _simplex[1].Vertice - _simplex[3].Vertice, _simplex[0].Vertice - _simplex[3].Vertice); break;
						}
						if (found) return (true);
					}
					else return (false);
				}
				_failed = true;
				return (false);
			}

			public bool EncloseOrigin()
			{
				switch (_order)
				{
					/* Point		*/
					case 0: break;
					/* Line			*/
					case 1:
						Vector3 ab = _simplex[1].Vertice - _simplex[0].Vertice;
						Vector3[] b ={	Vector3.Cross(ab, new Vector3(1, 0, 0)),
											Vector3.Cross(ab, new Vector3(0, 1, 0)),
											Vector3.Cross(ab, new Vector3(0, 0, 1)) };
						float[] m ={ b[0].LengthSquared(), b[1].LengthSquared(), b[2].LengthSquared() };
						Matrix r = Matrix.CreateFromQuaternion(new Quaternion(Vector3.Normalize(ab), _twoPi / 3));
						Vector3 w = b[m[0] > m[1] ? m[0] > m[2] ? 0 : 2 : m[1] > m[2] ? 1 : 2];
						Support(Vector3.Normalize(w), ref _simplex[4]); w = Vector3.TransformNormal(w, r);
                        Support(Vector3.Normalize(w), ref _simplex[2]); w = Vector3.TransformNormal(w, r);
                        Support(Vector3.Normalize(w), ref _simplex[3]); w = Vector3.TransformNormal(w, r);
						_order = 4;
						return true;
					/* Triangle		*/
					case 2:
						Vector3 n = Vector3.Normalize(Vector3.Cross(_simplex[1].Vertice - _simplex[0].Vertice, _simplex[2].Vertice - _simplex[0].Vertice));
						Support(n, ref _simplex[3]);
						Support(-n, ref _simplex[4]);
						_order = 4;
						return true;
					/* Tetrahedron	*/
					case 3: return (true);
					/* Hexahedron	*/
					case 4: return (true);
				}
				return (false);
			}
		}

		public class Epa
		{
			public class Face
			{
				public Gjk.MinkowskiVertice[] _vertices = new Gjk.MinkowskiVertice[3];
				public Face[] _faces = new Face[3];
				public int[] _e = new int[3];
				public Vector3 _n;
				public float _d;
				public int _mark;
				public Face _prev;
				public Face _next;
			}

			private Gjk _gjk;
			private Face _root;
			private int _nfaces;
			private int _iterations;
			private Vector3[,] _features = new Vector3[2, 3];
			private Vector3[] _nearest = new Vector3[2];
			private Vector3 _normal;
			private float _depth;
			private bool _failed;

			public Epa(Gjk gjk)
			{
				this._gjk = gjk;
			}

			public bool Failed { get { return _failed; } }
			public int Iterations { get { return _iterations; } }
			public Vector3 Normal { get { return _normal; } }
			public Vector3[] Nearest { get { return _nearest; } }

			public Vector3 GetCoordinates(Face face)
			{
				Vector3 o = face._n * -face._d;
				float[] a ={	Vector3.Cross(face._vertices[0].Vertice - o, face._vertices[1].Vertice - o).Length(),
								Vector3.Cross(face._vertices[1].Vertice - o, face._vertices[2].Vertice - o).Length(),
								Vector3.Cross(face._vertices[2].Vertice - o, face._vertices[0].Vertice - o).Length()};
				float sm = a[0] + a[1] + a[2];
				return (new Vector3(a[1], a[2], a[0]) / (sm > 0 ? sm : 1));
			}

			public Face FindBest()
			{
				Face bf = null;
				if (_root != null)
				{
					Face cf = _root;
					float bd = _infinity;
					do
					{
						if (cf._d < bd) { bd = cf._d; bf = cf; }
					} while (null != (cf = cf._next));
				}
				return bf;
			}

			public bool Set(ref Face f, Gjk.MinkowskiVertice a, Gjk.MinkowskiVertice b, Gjk.MinkowskiVertice c)
			{
				Vector3 nrm = Vector3.Cross(b.Vertice - a.Vertice, c.Vertice - a.Vertice);
				float len = nrm.Length();
				bool valid = (Vector3.Dot(Vector3.Cross(a.Vertice, b.Vertice), nrm) >= -_epaInFaceEpsilon &&
								 Vector3.Dot(Vector3.Cross(b.Vertice, c.Vertice), nrm) >= -_epaInFaceEpsilon &&
								 Vector3.Dot(Vector3.Cross(c.Vertice, a.Vertice), nrm) >= -_epaInFaceEpsilon);
				f._vertices[0] = a;
				f._vertices[1] = b;
				f._vertices[2] = c;
				f._mark = 0;
				f._n = nrm / (len > 0 ? len : _infinity);
				f._d = Max(0, -Vector3.Dot(f._n, a.Vertice));
				return valid;
			}

			public Face NewFace(Gjk.MinkowskiVertice a, Gjk.MinkowskiVertice b, Gjk.MinkowskiVertice c)
			{
				Face pf = new Face();
				if (Set(ref pf, a, b, c))
				{
					if (_root != null) _root._prev = pf;
					pf._prev = null;
					pf._next = _root;
					_root = pf;
					++_nfaces;
				}
				else
				{
					pf._prev = pf._next = null;
				}
				return (pf);
			}

			public void Detach(ref Face face)
			{
				if (face._prev != null || face._next != null)
				{
					--_nfaces;
					if (face == _root)
					{
						_root = face._next;
						_root._prev = null;
					}
					else
					{
						if (face._next == null)
						{
							face._prev._next = null;
						}
						else
						{
							face._prev._next = face._next;
							face._next._prev = face._prev;
						}
					}
					face._prev = face._next = null;
				}
			}

			public void Link(ref Face f0, int e0, ref Face f1, int e1)
			{
				f0._faces[e0] = f1; f1._e[e1] = e0;
				f1._faces[e1] = f0; f0._e[e0] = e1;
			}

			public Gjk.MinkowskiVertice Support(Vector3 w)
			{
				Gjk.MinkowskiVertice v = new Gjk.MinkowskiVertice();
				_gjk.Support(w, ref v);
				return v;
			}

			private static int[] mod3 ={ 0, 1, 2, 0, 1 };

			public int BuildHorizon(int markid, Gjk.MinkowskiVertice w, ref Face f, int e, ref Face cf, ref Face ff)
			{
				int ne = (0);
				if (f._mark != markid)
				{
					int e1 = (mod3[e + 1]);
					if ((Vector3.Dot(f._n, w.Vertice) + f._d) > 0)
					{
						Face nf = NewFace(f._vertices[e1], f._vertices[e], w);
						Link(ref nf, 0, ref f, e);
						if (cf != null) Link(ref cf, 1, ref nf, 2); else ff = nf;
						cf = nf; ne = 1;
					}
					else
					{
						int e2 = (mod3[e + 2]);
						Detach(ref f);
						f._mark = markid;
						ne += BuildHorizon(markid, w, ref f._faces[e1], f._e[e1], ref cf, ref ff);
						ne += BuildHorizon(markid, w, ref f._faces[e2], f._e[e2], ref cf, ref ff);
					}
				}
				return (ne);
			}

			public float EvaluatePD()
			{
				return EvaluatePD(_epaAccuracy);
			}

			private int[,] fidx;
			private int[,] eidx;

			public float EvaluatePD(float accuracy)
			{
				//Block* sablock = sa->BeginBlock();
				Face bestface = null;
				int markid = 1;
				_depth = -_infinity;
				_normal = new Vector3();
				_root = null;
				_nfaces = 0;
				_iterations = 0;
				_failed = false;
				/* Prepare hull		*/
				if (_gjk.EncloseOrigin())
				{
					int nfidx = 0;
					int neidx = 0;
					Gjk.MinkowskiVertice[] basemkv = new Gjk.MinkowskiVertice[5];
					Face[] basefaces = new Face[6];
					switch (_gjk.Order)
					{
						/* Tetrahedron		*/
						case 3:
							{
								fidx = new int[,] { { 2, 1, 0 }, { 3, 0, 1 }, { 3, 1, 2 }, { 3, 2, 0 } };
								eidx = new int[,] { { 0, 0, 2, 1 }, { 0, 1, 1, 1 }, { 0, 2, 3, 1 }, { 1, 0, 3, 2 }, { 2, 0, 1, 2 }, { 3, 0, 2, 2 } };
								nfidx = 4; neidx = 6;
							} break;
						/* Hexahedron		*/
						case 4:
							{
								fidx = new int[,] { { 2, 0, 4 }, { 4, 1, 2 }, { 1, 4, 0 }, { 0, 3, 1 }, { 0, 2, 3 }, { 1, 3, 2 } };
								eidx = new int[,] { { 0, 0, 4, 0 }, { 0, 1, 2, 1 }, { 0, 2, 1, 2 }, { 1, 1, 5, 2 }, { 1, 0, 2, 0 }, { 2, 2, 3, 2 }, { 3, 1, 5, 0 }, { 3, 0, 4, 2 }, { 5, 1, 4, 1 } };
								nfidx = 6; neidx = 9;
							} break;
					}
					int i;

					for (i = 0; i <= _gjk.Order; ++i)
					{
						//basemkv[i] = (GJK::Mkv*)sa->Allocate(sizeof(GJK::Mkv)); 
						basemkv[i] = new Gjk.MinkowskiVertice();
						basemkv[i].Vertice = _gjk.Simplex[i].Vertice;
						basemkv[i].Ray = _gjk.Simplex[i].Ray;
					}
					for (i = 0; i < nfidx; ++i)
					{
						basefaces[i] = NewFace(basemkv[fidx[i, 0]], basemkv[fidx[i, 1]], basemkv[fidx[i, 2]]);
					}
					for (i = 0; i < neidx; ++i)
					{
						Link(ref basefaces[eidx[i, 0]], eidx[i, 1], ref basefaces[eidx[i, 2]], eidx[i, 3]);
					}
				}
				if (0 == _nfaces)
				{
					return _depth;
				}
				/* Expand hull		*/
				for (; _iterations < _epaMaxIterations; ++_iterations)
				{
					Face bf = FindBest();
					if (bf != null)
					{
						Gjk.MinkowskiVertice w = Support(-bf._n);
						float d = Vector3.Dot(bf._n, w.Vertice) + bf._d;
						bestface = bf;
						if (d < -accuracy)
						{
							Face cf = null;
							Face ff = null;
							int nf = 0;
							Detach(ref bf);
							bf._mark = ++markid;
							for (int i = 0; i < 3; ++i)
							{
								nf += BuildHorizon(markid, w, ref bf._faces[i], bf._e[i], ref cf, ref ff);
							}
							if (nf <= 2) { break; }
							Link(ref cf, 1, ref ff, 2);
						}
						else break;
					}
					else break;
				}
				/* Extract contact	*/
				if (bestface != null)
				{
					Vector3 b = GetCoordinates(bestface);
					_normal = bestface._n;
					_depth = Max(0, bestface._d);
					for (int i = 0; i < 2; ++i)
					{
						float s = i != 0 ? -1 : 1;
						for (int j = 0; j < 3; ++j)
						{
							_features[i, j] = _gjk.LocalSupport(s * bestface._vertices[j].Ray, i);
						}
					}
					_nearest[0] = _features[0, 0] * b.X + _features[0, 1] * b.Y + _features[0, 2] * b.Z;
					_nearest[1] = _features[1, 0] * b.X + _features[1, 1] * b.Y + _features[1, 2] * b.Z;
				}
				else _failed = true;
				return _depth;
			}

			private float Max(float a, float b)
			{
				return (a > b ? a : b);
			}

			private float Min(float a, float b)
			{
				return (a < b ? a : b);
			}
		}
	}
}
