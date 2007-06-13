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

namespace XnaDevRu.BulletX
{
	public class UnionFind : IDisposable
	{
		private List<Element> _elements = new List<Element>();

		public int ElementCount
		{
			get { return _elements.Count; }
		}

		public void SortIslands()
		{
			for (int i = 0; i < _elements.Count; i++)
			{
				_elements[i].ID = Find(i);
				_elements[i].Size = i;
			}

			_elements.Sort(Sort);
		}

		private static int Sort(Element x, Element y)
		{
			if (x.ID < y.ID) return -1;
			//else if (x.ID > y.ID) return 1;
			else return 0;
		}

		public void Reset(int number)
		{
			Allocate(number);

			for (int i = 0; i < number; i++)
			{
				Element element = new Element();
				element.ID = i; 
				element.Size = 1;
				_elements.Insert(i, element);
			}
		}

		public bool IsRoot(int index)
		{
			return (_elements[index].Size == index);
		}

		public Element this[int index]
		{
			get { return _elements[index]; }
		}

		public void Allocate(int number)
		{
			//Does nothing
			_elements = new List<Element>(number);
		}

		public bool Find(int i, int j)
		{
			return (Find(i) == Find(j));
		}

		public int Find(int i)
		{
			while (i != _elements[i].ID)
			{
				//Element element = _elements[i];
				//element.ID = _elements[_elements[i].ID].ID;
				_elements[i].ID = _elements[_elements[i].ID].ID;
				i = _elements[i].ID;
			}

			return i;
		}

		public void Unite(int p, int q)
		{
			int i = Find(p), j = Find(q);
			if (i == j)
				return;

			//weighted quick union, this keeps the 'trees' balanced, and keeps performance of unite O( log(n) )
			//if (_elements[i].Size < _elements[j].Size)
			//{
			//    Element element = _elements[i];
			//    element.ID = j;
			//    _elements[i] = element;

			//    element = _elements[j];
			//    element.Size += _elements[i].Size;
			//    _elements[j] = element;
			//}
			//else
			//{
			//    Element element = _elements[j];
			//    element.ID = i;
			//    _elements[j] = element;

			//    element = _elements[i];
			//    element.Size += _elements[j].Size;
			//    _elements[i] = element;
			//}
			_elements[i].ID = j;
			_elements[j].Size += _elements[i].Size;
		}

		#region IDisposable Members

		public void Dispose()
		{
			_elements.Clear();
		}

		#endregion
	}

	public class Element
	{
		private int _id;
		private int _size;

		public int ID { get { return _id; } set { _id = value; } }
		public int Size { get { return _size; } set { _size = value; } }
	}
}