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

using System;
using System.Collections.Generic;
using System.Linq;
using System.Text;
using System.Threading.Tasks;

namespace AutoMesh
{
  class AM_Coedge
  {
    AM_Edge [] m_Edges = { null, null };
    int m_Index = -1;      // Indice corrispondente con Array globale

    internal AM_Coedge()
    {

    }

    internal AM_Coedge(AM_Vertex vtx_org, AM_Vertex vtx_dest)
    {
      m_Edges[0] = new AM_Edge(this, 0);
      m_Edges[1] = new AM_Edge(this, 1);

      m_Edges[0].Vertex = vtx_org;
      m_Edges[1].Vertex = vtx_dest;
    }

    internal AM_Edge this[int i]
    {
      get {
        return this.m_Edges[i];
      }

    }

    internal AM_Edge Edge()
    {
      return m_Edges[0];
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

  }
}
