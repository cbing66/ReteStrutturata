/*
MIT License

Copyright (c) 2005-2020 Alessandro Corazzin

Permission is hereby granted, free of charge, to any person obtaining a copy
of this software and associated documentation files (the "Software"), to deal
in the Software without restriction, including without limitation the rights
to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
copies of the Software, and to permit persons to whom the Software is
furnished to do so, subject to the following conditions:

The above copyright notice and this permission notice shall be included in all
copies or substantial portions of the Software.

THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
SOFTWARE.
*/

using Rhino.Geometry;
using System;
using System.Collections.Generic;
using System.Diagnostics;
using System.Linq;
using System.Text;
using System.Threading.Tasks;

namespace AutoMesh
{
  class AM_Vertex
  {
    Point2d m_Coord = Point2d.Unset;

    double m_Z = 0; // La coordinata Z è gestita separatemente dal punto di dominio R2
    double m_Space = 0;
    AM_Edge m_Edge = null;
    int m_Index = -1;  // corrispondenza con indice di m_ArrayVertexes
    int m_Flag = 0;

    internal AM_Vertex()
    {
    }

    internal Point2d Coord
    {
      get {
        return m_Coord;
      }

      set {
        m_Coord = value;
      }
    }

    internal Point3d Coord3d
    {
      get {
        return new Point3d(m_Coord.X, m_Coord.Y, m_Z);
      }
    }

    internal AM_Edge Edge
    {
      get {
        return m_Edge;
      }

      set {
        m_Edge = value;
      }
    }

    internal double Space
    {
      get {
        return m_Space;
      }
    }

    internal double Z
    {
      get {
        return m_Z;
      }

      set {
        m_Z = value;
      }
    }

    internal int Index
    {
      get {
        return m_Index;
      }

      set {
        m_Index = value;
      }
    }

    internal int Flag
    {
      get {
        return m_Flag;
      }

      set {
        m_Flag = value;
      }
    }

    internal AM_Vertex(Point2d coord, double z, double space)
    {
      m_Coord = coord;
      m_Z = z;
      m_Space = space;
    }

    internal bool IsEqual(Point2d coord, double toll)
    {
      return m_Coord.EpsilonEquals(coord, toll);
    }

    internal int NumDegree()
    {
      if (m_Edge == null)
        return 0;

      int degree = 0;

      AM_Edge pNext = m_Edge;
      do {
        degree++;
        pNext = pNext.Next;
      } while (pNext != m_Edge);
      return degree;
    }

    internal int NumConnectedFace()
    {
      if (m_Edge == null)
        return 0;

      int nFace = 0;

      AM_Edge pNext = m_Edge;
      do {
        if (pNext.CcwFace() != null)
          nFace++;
        pNext = pNext.Next;
      } while (pNext != m_Edge);

      return nFace;
    }

    internal double Degree(bool bSimple = false)
    {
      int degree = 0;
      if (m_Edge==null) {
        Debug.Assert(false);
        return 0;
      }

      // Il "grado" di un vertice è definito come il numero di vertici
      // ad esso conneesso.
      if (bSimple) {
        // Se bSimple = true si conta semplicemente il numero
        // di spigoli connessi

        AM_Edge pNext = m_Edge;
        do {
          degree++;
          pNext = pNext.Next;
        } while (pNext != m_Edge);
        return degree;
      }

      double angle = 0;
      {
        AM_Edge pNext = m_Edge;
        do {
          degree++;
          if (pNext.CcwFace() != null) {
            Vector2d v0 = (pNext.DestCoord() - m_Coord);
            Vector2d v1 = (pNext.Next.DestCoord() - m_Coord);
            v0.Unitize();
            v1.Unitize();

            double cos_ang = Vector2d.Multiply(v0, v1);
            angle += Math.Acos(cos_ang);
          }
          pNext = pNext.Next;
        } while (pNext != m_Edge);
      }

      return degree * 2 * Math.PI / angle;
    }

    internal AM_Edge FindEdge(AM_Vertex pVertex2)
    {
      AM_Edge pNext = m_Edge;
      do {
        if (pNext.Destination() == pVertex2)
          return pNext;
        pNext = pNext.Next;
      } while (pNext != m_Edge);
      return null;
    }
  }
}
