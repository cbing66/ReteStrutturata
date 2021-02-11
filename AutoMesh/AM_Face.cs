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
  class AM_Face
  {
    // Classificazione del tipo di faccia
    internal enum EFaceType { FT_NONE, FT_ACTIVE, FT_WAITING, FT_ACCEPTED };

    // --- Dati membro ---
    int m_Index = -1;   // corrispondenza con indice di m_ArrayFace
    int m_ActiveIndex = -1;
    EFaceType m_FaceType = EFaceType.FT_NONE;

    AM_Edge[] m_pEdges;
    Point2d m_CircumCenter;
    double m_CircumRadius = 0;

    internal AM_Face(int nEdges = 3)
    {
      m_pEdges = new AM_Edge[nEdges];
    }

    // Attributi
    internal AM_Edge this[int n]
    {
      get {
        return m_pEdges[n];
      }

      set {
        m_pEdges[n] = value;
      }
    }

    internal AM_Vertex Vertex(int n)
    {
      return m_pEdges[n].Vertex;
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

    internal int ActiveIndex
    {
      get {
        return m_ActiveIndex;
      }

      set {
        m_ActiveIndex = value;
      }
    }

    internal Vector3d Normal
    {
      get {
        Vector3d v1 = Vertex(1).Coord3d - Vertex(0).Coord3d;
        Vector3d v2 = Vertex(NumEdges - 1).Coord3d - Vertex(0).Coord3d;
        Vector3d norm = Vector3d.CrossProduct(v1, v2);
        norm.Unitize();
        return norm;
      }
    }

    internal Point2d CircumCenter
    {
      get {
        return m_CircumCenter;
      }
    }

    internal double CircumRadius
    {
      get {
        return m_CircumRadius;
      }
    }

    internal int NumEdges
    {
      get {
        return m_pEdges!= null? m_pEdges.Length : 0;
      }
    }

    internal EFaceType FaceType
    {
      get {
        return m_FaceType;
      }

      set {
        m_FaceType = value;
      }
    }

    bool CheckSwapEdge(AM_Edge pedge)
    {
      // Controlla l'ammissibilità dello swap.
      // Uno spigolo può essere scambiato all'interno di un quadrangolo
      // se quest'ultimo è convesso.

      Point2d p1 = pedge.CcwEdge().DestCoord();
      Point2d p2 = pedge.OrgCoord();
      Point2d p3 = pedge.DestCoord();
      Point2d p4 = pedge.Symm().CcwEdge().DestCoord();

      Vector2d v1 = p4-p2;
      Vector2d v2 = p1-p2;
      v1.Unitize();
      v2.Unitize();

      if ((v1.X * v2.Y - v2.X * v1.Y) < AM_Util.FLT_EPSILON)
        return false;

      Vector2d v3 = p1-p3;
      Vector2d v4 = p4-p3;
      v3.Unitize();
      v4.Unitize();

      if ((v3.X * v4.Y - v4.X * v3.Y) < AM_Util.FLT_EPSILON)
        return false;

      return true;
    }

    internal static void SwapEdge(AM_Edge pedge)
    {
      Debug.Assert(pedge.CcwFace().NumEdges == 3);
      Debug.Assert(pedge.CwFace().NumEdges == 3);

      AM_Edge pstart = pedge;
      do {
        Point2d p1 = pedge.CcwEdge().DestCoord();
        Point2d p2 = pedge.OrgCoord();
        Point2d p3 = pedge.DestCoord();
        Point2d p4 = pedge.Symm().CcwEdge().DestCoord();

        //int n1 = pedge.CcwEdge().Destination().Index;
        //int n2 = pedge.Origin().Index;
        //int n3 = pedge.Destination().Index;
        //int n4 = pedge.Symm().CcwEdge().Destination().Index;

        //int v1 = pedge.CcwEdge().Symm().m_nVertex;
        //int v2 = pedge.m_nVertex;
        //int v3 = pedge.Symm().m_nVertex;
        //int v4 = pedge.Symm().CcwEdge().Symm().m_nVertex;

        AM_Edge poldPrev = pedge.Prev;
        AM_Edge poldNext = pedge.Next;
        AM_Edge pnewNext = pedge.Prev.Symm();
        AM_Edge pnewPrev = pnewNext.Prev;
        AM_Face poldFace = pedge.Face;


        pedge.Origin().Edge = poldNext;
        pedge.Vertex = pnewNext.Vertex; // Ripristina l'origine...
        pedge.Origin().Edge = pedge;     // e aggiorna il suo puntatore

        // ripristina i collegamenti corretti
        poldPrev.Next = poldNext;
        poldNext.Prev = poldPrev;
        pnewPrev.Next = pedge;
        pnewNext.Prev = pedge;
        pedge.Next = pnewNext;
        pedge.Prev = pnewPrev;

        Debug.Assert(pedge.Origin() != pedge.Destination());

        // parte dallo spigolo che definisce la faccia sinistra
        // e che è precedente in senso antiorario
        pnewNext = poldNext.Symm();
        for (int i = 0; i < 3; i++) {
          pnewNext.Face = poldFace;
          poldFace[i] = pnewNext;
          pnewNext = pnewNext.CcwEdge();
        }

        pedge = pedge.Symm();

      } while (pstart != pedge);
    }

    // Connessioni
    internal bool SetConnection()
    {
      // testa la compatibilità delle facce. Eventuali problemi
      // possono insorgere se vi è incoerenza tra le normali di
      // due facce adiacenti
      for (int i = 0; i < NumEdges; i++) {
        if (m_pEdges[i].Face != null)
          return false;
      }

      Debug.Assert(NumEdges <= 4);

      // Inizializza le connessioni preesistenti
      AM_Edge [] poldConn = new AM_Edge[8];
      for (int i = 0; i < 8; i++)
        poldConn[i] = null;

      for (int i = 0; i < NumEdges; i++) {
        AM_Edge pedge = m_pEdges[i];
        AM_Edge psymm = pedge.Symm();
        pedge.Face = this;

        if (pedge.Next != null) {
          poldConn[i * 2] = pedge.Next;
        }
        pedge.Next = m_pEdges[(i + NumEdges - 1) % NumEdges].Symm();

        if (psymm.Prev != null) {
          poldConn[i * 2 + 1] = psymm.Prev;
        }
        psymm.Prev = m_pEdges[(i + 1) % NumEdges];
      }

      if (!AdjustConnection(poldConn)) {
        // la connessione è fallita: si ripristinano
        // le connessioni precedenti
        for (int i = 0; i < NumEdges; i++) {
          AM_Edge pedge = m_pEdges[i];
          AM_Edge psymm = pedge.Symm();
          pedge.Face = null;

          if (poldConn[i * 2] != null) {
            pedge.Next = poldConn[i * 2];
          } else {
            pedge.Next = null;
          }

          if (poldConn[i * 2 + 1] != null) {
            psymm.Prev = poldConn[i * 2 + 1];
          } else {
            psymm.Prev = null;
          }
        }

        return false;
      }
      return true;
    }

    // --- Attributi ---
    internal AM_Face GetNearTriangle(int i)
    {
      return (AM_Face)m_pEdges[i].Symm().Face;
    }


    internal AM_Edge GetStartingEdge()
    {
      return m_pEdges[0];
    }

    // --- Test ---
    internal double TeoricRadius(Point2d p, AM_Mesh2d pSpaceFunction)
    {
      double f = SpaceFunction(p, pSpaceFunction);
      return (AM_Mesh2d.RadCoef * f);
    }

    internal double SpaceFunction(Point2d p, AM_Mesh2d pSpaceFunction)
    {
      if (pSpaceFunction != null) {
        double z = 0;
        if (pSpaceFunction.GetCoordZ(p, ref z))
          return z;
      }

      Point2d p0 = Vertex(0).Coord;
      Point2d p1 = Vertex(1).Coord;
      Point2d p2 = Vertex(2).Coord;

      double det = AM_Util.TriArea(p0, p1, p2);
      Debug.Assert(det != 0);
      double alfa = AM_Util.TriArea(p0, p, p2) / det;
      double beta = AM_Util.TriArea(p0, p1, p) / det;

      double f0 = Vertex(0).Space;
      double f1 = Vertex(1).Space;
      double f2 = Vertex(2).Space;

      double f = f0 + alfa * (f1 - f0) + beta * (f2 - f0);
      return f;
    }

    internal bool SpacingTest(Point2d p, AM_Mesh2d pSpaceFunction)
    {
      double f = SpaceFunction(p, pSpaceFunction);
      double dist = 0;

      for (int i = 0; i < 3; i++) {
        Vector2d v = Vertex(i).Coord - p;
        dist = v.Length;
        if (dist < (AM_Mesh2d.SpaceCoef * f))
          return false;
      }

      return true;
    }

    internal bool InnerTest(Point2d p)
    {
      Point2d x = p;

      // Per appartenere alla faccia devono essere tutte a sinistra dello spigolo
      for (int i = 0; i < 3; i++) {
        AM_Edge e = m_pEdges[i];
        if (x.EpsilonEquals(e.OrgCoord(), AM_Util.FLT_EPSILON))
          return true;

        if (AM_Edge.RightOf(x, e))
          return false;
      }

      return true;
    }

    // --- Impostazioni ---
    internal Point2d InsertPoint(AM_Mesh2d pSpaceFunction)
    {
      AM_Edge pRif = null;
      for (int i = 0; i < 3; i++) {
        if (GetNearTriangle(i) == null) {
          pRif = m_pEdges[i];
          break;
        }

        if (GetNearTriangle(i).m_FaceType == EFaceType.FT_ACCEPTED) {
          pRif = m_pEdges[i];
        }
      }

      if (pRif == null) {
        return Vertex(0).Coord;
      }

      Point2d thirdPoint = Point2d.Origin;
      Point2d midPoint = 0.5 * (pRif.OrgCoord() + pRif.DestCoord());
      Point2d directionPoint = m_CircumCenter;

      for (int i = 0; i < 3; i++) {
        if (Vertex(i) != pRif.Origin() &&
            Vertex(i) != pRif.Destination()) {
          thirdPoint = Vertex(i).Coord;
          break;
        }
      }

      if (m_CircumCenter == midPoint) { //triangolo rettangolo
        directionPoint = thirdPoint;
      }

      double radius = TeoricRadius(midPoint, pSpaceFunction);
      double p = (pRif.OrgCoord() - pRif.DestCoord()).Length / 2;
      double q = (midPoint - m_CircumCenter).Length;

      if (radius < p)
        radius = p;
      if (q != 0) {
        double tmp = (p * p + q * q) / (2* q);
        if (radius > tmp)
          radius = tmp;
      }

      Vector2d versor;
      if (AM_Edge.LeftOf(directionPoint, pRif) && AM_Edge.RightOf(thirdPoint, pRif) ||
          AM_Edge.LeftOf(thirdPoint, pRif) && AM_Edge.RightOf(directionPoint, pRif)) {
        versor = midPoint - directionPoint;
      } else {
        versor = directionPoint - midPoint;
      }
      versor.Unitize();

      double d = radius + Math.Sqrt(radius * radius - p * p);
      Point2d point = midPoint + d * versor;

      return point;
    }

    internal void SetTriangleParameter(AM_Mesh2d pSpaceFunction)
    {
      ComputeCircumCircle();

      Point2d GravityCenter = 1/3d* (Vertex(0).Coord
                                     + Vertex(1).Coord
                                     + Vertex(2).Coord);
      double r = TeoricRadius(GravityCenter, pSpaceFunction);

      if ((r / m_CircumRadius) > AM_Mesh2d.Delta) {
        m_FaceType = EFaceType.FT_ACCEPTED;
      } else {
        m_FaceType = EFaceType.FT_NONE;
      }
    }

    internal void ComputeCircumCircle()
    {
      Point2d P1 = Vertex(0).Coord,
              P2 = Vertex(1).Coord,
              P3 = Vertex(2).Coord;

      AM_Util.CircumCircle(P1, P2, P3, ref m_CircumCenter, ref m_CircumRadius);
    }

    bool AdjustConnection(AM_Edge[] poldConn)
    {
      for (int i = 0; i < NumEdges; i++) {
        AM_Edge pconnSx = poldConn[i * 2];
        AM_Edge pconnDx = poldConn[(i * 2 + (NumEdges * 2) - 1) % (NumEdges * 2)];


        if (pconnDx == null && pconnSx == null) {
          if (Vertex(i).Edge != null) {
            // Inserisce i due spigoli della faccia nell'anello del vertice
            // individuato da Vertex(i)

            AM_Edge pedge = ((AM_Edge)(Vertex(i).Edge)).FindLastConnected();
            Debug.Assert(pedge != null);
            AM_Edge poldnextEdge = pedge.Next;

            m_pEdges[i].Prev = pedge;
            pedge.Next = m_pEdges[i];
            poldnextEdge.Prev = m_pEdges[i].Next;
            m_pEdges[i].Next.Next = poldnextEdge;

          } else {
            // Lo spigolo è isolato: inizializza l'anello

            m_pEdges[i].Prev = m_pEdges[i].Next;
            m_pEdges[i].Next.Next = m_pEdges[i];
            Vertex(i).Edge = m_pEdges[i];
          }
        } else {
          if (pconnSx == pconnDx)
            continue;
          // vi erano connessioni preesistenti: l'anello viene aggiornato
          if (pconnSx != null) { // anello in senso antiorario
            AM_Edge padjust = pconnSx;
            AM_Edge pstart = pconnSx.Prev;

            /*TRACE_EDGE(padjust);
            TRACE_EDGE(pstart);*/

            int ncheck = 0; //contatore per evitare loop infinito
            while (padjust != null) {
              if (ncheck++ > 300) {
                Debug.Assert(false);
                return false;
              }

              if (padjust.Prev.Next == padjust)
                break;
              AM_Edge pnewconn = padjust;
              padjust.Prev = pstart.FindLastConnected();
              pstart = padjust.Prev;
              padjust = pstart.Next;
              pstart.Next = pnewconn;
            }
          }

          if (pconnDx != null) { // anello in senso orario
            AM_Edge padjust = pconnDx;
            AM_Edge pstart = pconnDx.Next;

            /*TRACE_EDGE(padjust);
            TRACE_EDGE(pstart);*/
            int ncheck = 0; //contatore per evitare loop infinito

            while (padjust != null) {
              if (ncheck++ > 300) {
                Debug.Assert(false);
                return false;
              }

              if (padjust.Next.Prev == padjust)
                break;
              AM_Edge pnewconn = padjust;
              padjust.Next = pstart.FindLastConnected(false);
              pstart = padjust.Next;
              padjust = pstart.Prev;
              pstart.Prev = pnewconn;
            }
          }
        }
      }

      return true;
    }


  }
}
