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
  class AM_Boundary
  {
    // --- Dati membro ---
    bool m_bFlagHole = false;         // Indica se è un buco

    // --- Dati membro ---
    List<Point3d> m_ArrayCoord = new List<Point3d>();     // Array geometrico del contorno (.Z -> Space)
    List<AM_Vertex> m_ArrayVertex = new List<AM_Vertex>();    // Array dei vertici generati
    List<int> m_GenVertexArray = new List<int>(); //
    int m_LoopIndex = -1;      // Corrispondenza con il loop di GeoRegion

    internal void SetLoopIndex(int index)
    {
      m_LoopIndex = index;
    }

    internal int GetLoopIndex()
    {
      return m_LoopIndex;
    }

    internal bool FlagHole
    {
      get {
        return m_bFlagHole;
      }

      set {
        m_bFlagHole = value;
      }
    }

    // --- Costruttori ---
    internal AM_Boundary()
    {

    }

    internal AM_Boundary(List<Point2d> arrayCoord)
    {
      m_LoopIndex = -1;
      In(arrayCoord);
    }

    // --- Attributi ---
    internal int GetNumVertex()
    {
      return m_ArrayCoord.Count();
    }

    internal Point3d this[int i]
    {
      get {
        return m_ArrayCoord[i];
      }
    }

    internal AM_Vertex Vertex(int i)
    {
      return m_ArrayVertex[i];
    }

    internal int GetNumGenVertex()
    {
      return m_GenVertexArray.Count();
    }

    internal int GenMeshVertexIndex(int i)
    {
      return m_GenVertexArray[i];
    }

    internal AM_Edge GetGenEdge(int i)
    {
      int numVertex = GetNumGenVertex();
      if (i >= numVertex) {
        Debug.Assert(false);
        return null;
      }

      AM_Vertex pV1 = m_ArrayVertex[i];
      AM_Vertex pV2 = m_ArrayVertex[(i + 1) % numVertex];

      AM_Edge pEdge = pV1.FindEdge(pV2); ;
      return pEdge;
    }

    // --- Copia ---
    AM_Boundary CopyBoundary(AM_Mesh2d source, AM_Mesh2d dest,
                             bool bSameGenVertex = true)
    {
      AM_Boundary pCopy = new AM_Boundary();

      pCopy.m_LoopIndex = m_LoopIndex;

      if (bSameGenVertex) {
        // Il numero dei vertici generati è lo stesso;
        // viene normalmente usato in caso di 'merge' tra due mesh adiacenti
        pCopy.m_ArrayCoord.Capacity = m_ArrayCoord.Count;
        for (int i = 0; i < m_ArrayCoord.Count; i++) {
          pCopy.m_ArrayCoord[i] = m_ArrayCoord[i];
        }

        pCopy.m_GenVertexArray.Capacity = m_GenVertexArray.Count;
        for (int i = 0; i < m_GenVertexArray.Count; i++) {
          pCopy.m_GenVertexArray[i] = m_GenVertexArray[i];
        }

        // Trova il primo vertice
        int destVertex = dest.ArrayVertexes.Count;
        int nVertex = dest.AddVertex(new Point2d(m_ArrayCoord[0]), m_ArrayCoord[0].Z);
        Debug.Assert(nVertex < destVertex); // non vengono aggiunti vertici

        AM_Vertex pVertex = dest.ArrayVertexes[nVertex];
        pVertex.Flag |= 0x01;
        pCopy.m_ArrayVertex.Add(pVertex);

        for (int i = 1; i < m_ArrayCoord.Count; i++) {
          Point2d ptDest = new Point2d(m_ArrayCoord[i]);
          AM_Edge pEdge = pVertex.Edge;
          AM_Edge pNextEdge = pEdge.Next;

          while ((ptDest - pNextEdge.DestCoord()).Length > AM_Util.FLT_EPSILON) {
            if (pNextEdge == pEdge) {
              Debug.Assert(false);
              nVertex = dest.AddVertex(ptDest, 0);
              Debug.Assert(nVertex < destVertex);
              pNextEdge = dest.ArrayVertexes[nVertex].Edge.Symm();
              break;
            }
            pNextEdge = pNextEdge.Next;
          }

          pVertex = pNextEdge.Destination();
          pVertex.Flag |= 0x01;
          pCopy.m_ArrayVertex.Add(pVertex);
        }

        Debug.Assert(pCopy.m_ArrayVertex.Count == m_ArrayVertex.Count);
      } else {
        // Il numero dei vertici generati è diverso;
        // viene normalmente usato in caso di ricostruzione di contorni
        pCopy.m_GenVertexArray.Capacity = m_GenVertexArray.Count;

        for (int i = 1; i < m_GenVertexArray.Count; i++) {
          Point2d p0 = new Point2d(m_ArrayCoord[m_GenVertexArray[i - 1]]);
          Point2d p1 = new Point2d(m_ArrayCoord[m_GenVertexArray[i]]);

          AM_Vertex pV0 = dest.RangeSearch.Search(p0.X, p0.Y);
          AM_Vertex pV1 = dest.RangeSearch.Search(p1.X, p1.Y);
          Debug.Assert(pV0 != null && pV1 != null);

          // La direzione è data dal vettore p0-p1
          Vector2d vDir = (p1 - p0);
          vDir.Unitize();

          pCopy.m_GenVertexArray[i - 1] = pCopy.m_ArrayCoord.Count;

          AM_Vertex pV = pV0;
          while (pV != pV1) {
            pCopy.m_ArrayCoord.Add(new Point3d(pV.Coord.X, pV.Coord.Y, 0));
            pCopy.m_ArrayVertex.Add(pV);

            // Trova il vertice successivo
            double minCos = -double.MaxValue;

            AM_Edge pEdge = pV.Edge;
            AM_Edge pDirEdge = null;

            do {
              double dirCos = pEdge.GetVersor() * vDir;
              if (dirCos > minCos) {
                minCos = dirCos;
                pDirEdge = pEdge;
              }

              pEdge = pEdge.Next;
            } while (pEdge != pV.Edge);

            Debug.Assert(AM_Util.IsEqual(minCos, 1));
            pV = pDirEdge.Destination();
          }

        }
      }

      return pCopy;
    }

    // --- Impostazioni ---
    internal void In(List<Point2d> arrayCoord)
    {
      int size = arrayCoord.Count;
      m_ArrayCoord.Capacity = size;

      for (int i = 0; i < size; i++) {
        m_ArrayCoord[i] = new Point3d(arrayCoord[i].X, arrayCoord[i].Y, 0);
      }

      if (m_ArrayCoord[0] == m_ArrayCoord[m_ArrayCoord.Count - 1])
        m_bFlagHole = true;
    }

    internal void AddPoint(Point2d point, double s, bool bGenVertex = false)
    {
      int index = m_ArrayCoord.Count;
      m_ArrayCoord.Add(new Point3d(point.X, point.Y, s));
      if (bGenVertex) {
        m_GenVertexArray.Add(index);
      }
    }

    internal bool InsertIntoMesh(AM_Mesh2d pmesh)
    {
      AM_Vertex pvertex = null;

      for (int i = 0; i < m_ArrayCoord.Count; i++) {
        Point3d pt = m_ArrayCoord[i];
        if (pmesh.InsertPoint(new Point2d(pt), pt.Z, out pvertex)) {
          pvertex.Flag = 0x01;
          m_ArrayVertex.Add(pvertex);
        } else {
          /*
          Debug.Assert(false);
          throw 2;
          */
          pvertex.Flag = 0x01;
          m_ArrayVertex.Add(pvertex);

          // Il vertice esiste già: viene eliminato dalla sequenza
          //m_ArrayCoord.RemoveAt(i);
        }
      }
      return true;
    }
    internal void InsertVertex(int index, AM_Vertex pvertex)
    {
      pvertex.Flag = 0x01;
      m_ArrayCoord.Insert(index, new Point3d(pvertex.Coord.X, pvertex.Coord.Y, 0));
      m_ArrayVertex.Insert(index, pvertex);
      for (int i = 0; i < m_GenVertexArray.Count; i++) {
        if (m_GenVertexArray[i] >= index) {
          for (int k = i; k < m_GenVertexArray.Count; k++) {
            m_GenVertexArray[k] += 1;
          }
          break;
        }
      }

    }

    internal bool InsertVertex(AM_Mesh2d mesh, Point3d pt, int numGenVertex = 0)
    {
      while (numGenVertex < m_GenVertexArray.Count) {
        int v1 = m_GenVertexArray[numGenVertex];
        int v2 = m_GenVertexArray[(numGenVertex + 1) % m_GenVertexArray.Count];

        if (v2 < v1)
          v2 += m_GenVertexArray.Count;

        Point2d p = new Point2d(pt);
        Point2d p1 = new Point2d(m_ArrayCoord[v1]);
        Point2d p2 = new Point2d(m_ArrayCoord[v2]);

        Vector2d vec1 = (p2 - p1);
        Vector2d vec2 = (p - p1);

        vec1.Unitize();
        vec2.Unitize();

        if (AM_Util.IsEqual(vec2.Length, 0)) {
          double t = (p - p1).Length;
          for (int i = v1 + 1; i <= v2; i++) {
            Point2d pv = new Point2d(m_ArrayCoord[i % m_ArrayCoord.Count]);
            if ((pv- p1).Length > t) {
              AM_Vertex pVertex = null;
              if (mesh.InsertPoint(p, pt.Z, out pVertex)) {
                InsertVertex(i, pVertex);
                return true;
              } else
                return false;
            }
          }
        }

        numGenVertex++;
      }

      return false;
    }

    internal bool RecoverGenEdge(AM_Mesh2d mesh, int num, List<Point3d> AddArray, bool bStraight = false)
    {
      // se viene inserito un punto per aggiustare la conformità
      // il flag baddFlag diventa true
      bool baddFlag = false;

      if (!m_bFlagHole && num >= m_GenVertexArray.Count)
        return true;

      int v1 = m_GenVertexArray[num];
      int v2 = m_GenVertexArray[(num + 1) % m_GenVertexArray.Count];

      if (v2 < v1)
        v2 += GetNumVertex();

      AM_Edge pbase;
      AM_Edge pprev = pbase = Vertex(v1).Edge;

      for (int i = v1 + 1; i <= v2; i++) {
        AM_Vertex pV1 = Vertex((i - 1) % (GetNumVertex()));
        AM_Vertex pV2 = Vertex((i) % (GetNumVertex()));

        Point2d orgCoord = pV1.Coord;

        // si controlla che tutti i vertici siano in sequenza
        while (true) {
          Point2d baseCoord = pbase.DestCoord();

          Point2d prvCoord = new Point2d ( m_ArrayCoord[(i - 1)%(GetNumVertex())] );
          Point2d destCoord = new Point2d ( m_ArrayCoord[i%(GetNumVertex())]);

          if (baseCoord == destCoord) {
            // il vertice è in sequenza: si continua con il successivo
            break;
          } else {
            pbase = pbase.Next;

            if (pbase == pprev) {
              // il ciclo dell'anello si è chiuso senza trovare il vertice
              // successivo; è necessario inserire un vertice in mezzeria del
              // lato mancante

              if (!bStraight) {

                // 1. Algoritmo di ripristino del bordo con l'aggiunta del punto medio
                baddFlag = true;  // si segnala l'aggiunta di un vertice

                Point3d p1 = m_ArrayCoord[i-1];
                Point3d p2 = (m_ArrayCoord[i%(GetNumVertex())]);
                Point3d mid = 0.5 * (p1 + p2);
                Point3d insPt = new Point3d(mid.X, mid.Y, 0 );

                // si inserisce un vertice nel mezzo del
                AM_Vertex pvertex;
                mesh.InsertPoint(new Point2d(insPt), insPt.Z, out pvertex);

                if (pvertex == null) {
                  Debug.Assert(false);
                  //throw 6;
                }


                InsertVertex(i, pvertex);
                v2++;
                AddArray.Add(insPt);

                // si ricomincia il controllo
                pbase = Vertex(i-1).Edge;
                pprev = pbase;
              } else {
                // 2. Algoritmo di ripristino del bordo con swap di spigoli
                AM_Edge pdest = Vertex(i).Edge;

                Vector2d dir = destCoord - orgCoord;
                dir.Unitize();

                var m = AM_Util.AffineMatrix(orgCoord, dir);

                while (pV1.FindEdge(pV2) == null) {
                  bool bCoinc = false;
                  AM_Edge pSearch = pbase;

                  // Si controllano situazioni di appartenenza al lato da ripristinare
                  do {
                    double cosang = Vector2d.Multiply(pSearch.GetVersor(), dir);

                    if (AM_Util.IsEqual(cosang, 1, AM_Util.FLT_EPSILON)) {
                      // Lo spigolo appartiene già al lato da ripristinare
                      InsertVertex(i, pSearch.Destination());
                      v2++;

                      Point2d dc = pSearch.DestCoord();
                      AddArray.Add(new Point3d(dc.X, dc.Y, 0));

                      // si ricomincia il controllo
                      pbase = Vertex(i-1).Edge;
                      pprev = pbase;

                      bCoinc = true;
                      break;
                    }
                    pSearch = pSearch.Next;
                  } while (pSearch != pbase);

                  if (bCoinc)
                    break;

                  // Trova il lato di partenza
                  pSearch = pbase;

                  while (!AM_Util.IsInside(pSearch.GetVector(), pSearch.Next.GetVector(), dir)) {
                    pSearch = pSearch.Next;
                    if (pSearch == pprev) {
                      Debug.Assert(false);
                      //mesh.ExportMesh("RecoverSt7.txt");
                      return false;
                    }
                  }

                  AM_Edge pStartEdge = pSearch.CcwEdge();
                  List<AM_Edge> swapArray = new List<AM_Edge>();

                  while (pStartEdge.Destination() != pV2) {
                    Point2d o = pStartEdge.OrgCoord();
                    Point2d d = pStartEdge.DestCoord();
                    swapArray.Add(pStartEdge);

                    pStartEdge = pStartEdge.Prev;
                    Point2d pt = AM_Util.ToLocal(m, pStartEdge.DestCoord());
                    if (pt.Y< -AM_Util.FLT_EPSILON) {
                      pStartEdge = pStartEdge.CcwEdge();

                      Debug.Assert(AM_Util.ToLocal(m, pStartEdge.DestCoord()).Y>0);
                    }
                  }

                  for (int j=0; j<swapArray.Count; j++) {
                    AM_Edge pSwapEdge = swapArray[j];

                    // Vengono ruotati gli spigoli all'interno
                    if (AM_Util.CheckSwapEdge(pSwapEdge)) {

                      Debug.Assert(pSearch.CcwFace() != null && pSearch.Next.CwFace() != null);
                      Debug.Assert(pSwapEdge.CcwFace() != null && pSwapEdge.CwFace() != null);

                      AM_Face.SwapEdge(pSwapEdge);
                    }
                  }
                }
              }
            }
          }
        }
        pbase = Vertex(i%(GetNumVertex())).Edge;
        pprev = pbase;
      }

      return baddFlag;
    }
  }
}
