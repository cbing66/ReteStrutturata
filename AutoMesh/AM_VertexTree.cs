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
  class AM_VertexTree
  {
    class Node
    {
      internal AM_Vertex m_Vertex = null;
      internal Node m_Left = null;
      internal Node m_Right = null;

      internal Node() {}
    };

    Node m_Root = null;

    internal AM_VertexTree()
    {
      m_Root = null;
    }

    internal void Clear()
    {
      if (m_Root != null) {
        Delete(m_Root);
        m_Root = null;
      }
    }

    void Delete(Node root)
    {
      if (root.m_Left != null) {
        Delete(root.m_Left);
        root.m_Left = null;
      }

      if (root.m_Right != null) {
        Delete(root.m_Right);
        root.m_Right = null;
      }
    }

    internal bool Insert(AM_Vertex pVertex)
    {
      if (m_Root == null) {
        m_Root = new Node();
        m_Root.m_Vertex = pVertex;
        return true;
      }

      Node pNode = m_Root;
      Node pParent = null;
      bool bLeft = false;
      for (int level = 0; pNode != null; level++) {
        Debug.Assert(pNode.m_Vertex != null);
        if (pNode.m_Vertex.Coord.EpsilonEquals(pVertex.Coord, AM_Util.FLT_EPSILON)) {
          // Il vertice esiste già
          return false;
        }

        pParent = pNode;
        if ((level & 1)>0) {
          // Livello dispari: direzione x-x
          if (pVertex.Coord.X < pNode.m_Vertex.Coord.X) {
            pNode = pNode.m_Left;
            bLeft = true;
          } else {
            pNode = pNode.m_Right;
            bLeft = false;
          }
        } else {
          // Livello pari: direzione y-y
          if (pVertex.Coord.Y < pNode.m_Vertex.Coord.Y) {
            pNode = pNode.m_Left;
            bLeft = true;
          } else {
            pNode = pNode.m_Right;
            bLeft = false;
          }
        }
      }

      pNode = new Node();
      pNode.m_Vertex = pVertex;
      if (bLeft)
        pParent.m_Left = pNode;
      else
        pParent.m_Right = pNode;

      return true;
    }

    internal AM_Vertex Search(double x, double y)
    {
      Node pNode = m_Root;
      for (int level = 0; pNode != null; level++) {
        if (pNode.m_Vertex.Coord.EpsilonEquals(new Point2d(x,y), AM_Util.FLT_EPSILON)) {
          return pNode.m_Vertex;
        }

        if ((level & 1)>0) {
          // Livello dispari: direzione x-x
          if (x < pNode.m_Vertex.Coord.X) {
            pNode = pNode.m_Left;
          } else {
            pNode = pNode.m_Right;
          }
        } else {
          // Livello pari: direzione y-y
          if (y < pNode.m_Vertex.Coord.Y) {
            pNode = pNode.m_Left;
          } else {
            pNode = pNode.m_Right;
          }
        }
      }

      return null;
    }

  }
}
