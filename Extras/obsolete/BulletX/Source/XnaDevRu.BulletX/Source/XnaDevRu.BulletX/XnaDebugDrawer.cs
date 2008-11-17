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
using Microsoft.Xna.Framework.Graphics;

namespace XnaDevRu.BulletX
{
	public class XnaDebugDraw : IDebugDraw
	{
		DebugDrawModes _debugDrawModes = DebugDrawModes.NoDebug;
		GraphicsDevice _graphicsDevice;
		List<VertexPositionColor> _vertices = new List<VertexPositionColor>();
		VertexPositionColor[] _lineVertices = new VertexPositionColor[2];
        BasicEffect _basicEffect;

		public XnaDebugDraw(GraphicsDevice device)
		{
			this._graphicsDevice = device;
            this._basicEffect = new BasicEffect(device, null);
            this._graphicsDevice.VertexDeclaration = new VertexDeclaration(device, VertexPositionColor.VertexElements);
		}

		public void DrawAabb(Vector3 from, Vector3 to, Vector3 color)
		{
			Vector3 halfExtents = (to - from) * 0.5f;
			Vector3 center = (to + from) * 0.5f;

			Vector3 edgecoord = new Vector3(1f, 1f, 1f), pa, pb;
			for (int i = 0; i < 4; i++)
			{
				for (int j = 0; j < 3; j++)
				{
					pa = new Vector3(edgecoord.X * halfExtents.X, edgecoord.Y * halfExtents.Y,
						edgecoord.Z * halfExtents.Z);
					pa += center;

					int othercoord = j % 3;
					MathHelper.SetElement(ref edgecoord, othercoord, MathHelper.GetElement(edgecoord, othercoord) * -1f);
					pb = new Vector3(edgecoord.X * halfExtents.X, edgecoord.Y * halfExtents.Y,
						edgecoord.Z * halfExtents.Z);
					pb += center;

					DrawLine(pa, pb, color);
				}
				edgecoord = new Vector3(-1f, -1f, -1f);
				if (i < 3)
					MathHelper.SetElement(ref edgecoord, i, MathHelper.GetElement(edgecoord, i) * -1f);
			}
		}

        public void Update(Matrix view, Matrix projection)
        {
            this._basicEffect.AmbientLightColor = new Vector3(1, 1, 1);

            this._basicEffect.View = view;
            this._basicEffect.Projection = projection;
        }

		/// <summary>
		/// Put this between effect.
		/// </summary>
		public void DrawAll()
		{
            this._basicEffect.Begin();
            foreach (EffectPass pass in this._basicEffect.CurrentTechnique.Passes)
            {
                pass.Begin();

                if (_vertices.Count > 1)
                {
                    _graphicsDevice.DrawUserPrimitives<VertexPositionColor>(PrimitiveType.LineList, _vertices.ToArray(), 0, _vertices.Count / 2);
                    _vertices.Clear();
                }
                if (_lineVertices[0] != default(VertexPositionColor))
                    _graphicsDevice.DrawUserPrimitives<VertexPositionColor>(PrimitiveType.LineList, _lineVertices, 0, 1);

                pass.End();
            }
            this._basicEffect.End();
		}

		#region IDebugDraw Members

		public void DrawLine(Vector3 from, Vector3 to, Vector3 color)
		{
			if ((int)_debugDrawModes > 0)
			{
                this._basicEffect.Begin();
                foreach (EffectPass pass in this._basicEffect.CurrentTechnique.Passes)
                {
                    pass.Begin();

                    _lineVertices[0] = new VertexPositionColor(from, new Color(color));
                    _lineVertices[1] = new VertexPositionColor(to, new Color(color));
                    
                    _graphicsDevice.DrawUserPrimitives<VertexPositionColor>(PrimitiveType.LineList, _lineVertices, 0, 1);

                    pass.End();
                }
                this._basicEffect.End();
            
            }
		}

		public void DrawContactPoint(Vector3 pointOnB, Vector3 normalOnB, float distance, int lifeTime, Vector3 color)
		{
			if ((_debugDrawModes & DebugDrawModes.DrawContactPoints) != 0)
			{
				Vector3 to = pointOnB + normalOnB * distance;
				Vector3 from = pointOnB;
				//device.RenderState.PointSize = 10; // TODO: Experiment with this
				DrawLine(from, to, color);
			}
		}

		public DebugDrawModes DebugMode { get { return _debugDrawModes; } set { _debugDrawModes = value; } }

		#endregion
	}
}
