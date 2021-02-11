using Rhino.Geometry;
using System;
using System.Collections.Generic;
using System.Diagnostics;
using System.Linq;
using System.Text;
using System.Threading.Tasks;

namespace AutoMesh
{
  class AM_Edge
  {
    // Dati membro
    AM_Vertex m_Vertex = null;  // vertice di origine (su array globale)
    AM_Edge m_pNext = null;  // prossimo vertice in senso antiorario
    AM_Edge m_pPrev = null;  // vertice precedente
    AM_Face m_pFace = null;  // faccia a sx vista dall'origine
    int m_Flag = 0;   // 0x01 : per distinguere la dualità
    // 0x02 : per appartenenza a contorno

    AM_Coedge m_WingedEdge = null;

    internal AM_Vertex Vertex
    {
      get {
        return m_Vertex;
      }
      set {
        m_Vertex = value;
      }
    }

    internal AM_Edge Next
    {
      get {
        return m_pNext;
      }
      set {
        m_pNext = value;
      }
    }

    internal AM_Edge Prev
    {
      get {
        return m_pPrev;
      }
      set {
        m_pPrev = value;
      }
    }

    internal AM_Face Face
    {
      get {
        return m_pFace;
      }
      set {
        m_pFace = value;
      }
    }

    internal AM_Coedge WingedEdge
    {
      get {
        return m_WingedEdge;
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

    internal AM_Edge(AM_Coedge coedge, int index)
    {
      Debug.Assert(coedge != null);
      Debug.Assert(index==0 || index == 1);
      m_WingedEdge = coedge;
      m_Flag |= index;
    }

    // --- Attributi ---
    internal AM_Vertex DestVertex()
    {
      return Symm().Vertex;
    }


    internal AM_Edge Symm()
    {
      return (m_Flag & 1)>0 ? m_WingedEdge[0] : m_WingedEdge[1];
    }

    internal AM_Vertex Origin()
    {
      return m_Vertex;
    }

    internal AM_Vertex Destination()
    {
      return Symm().Origin();
    }

    internal AM_Edge CcwEdge()
    {
      return Symm().Prev;
    }

    internal AM_Edge CwEdge()
    {
      return m_pNext.Symm();
    }

    internal Point2d OrgCoord()
    {
      return m_Vertex.Coord;
    }

    internal Point2d DestCoord()
    {
      return DestVertex().Coord;
    }

    internal AM_Face CwFace()
    {
      return Symm().Face;
    }

    internal AM_Face CcwFace()
    {
      return m_pFace;
    }

    internal Vector2d GetVector()
    {
      return Destination().Coord - Origin().Coord;
    }

    internal Vector2d GetVersor()
    {
      Vector2d v = GetVector();
      v.Unitize();
      return v;
    }

    internal AM_Edge FindLastConnected(bool ccw = true)
    {
      AM_Edge pedge;
      AM_Edge psentry;
      if (ccw) { // trova il primo spigolo connesso in senso antiorario
        pedge = m_pNext;
        psentry = this;   // da rivedere la sentinella
        Debug.Assert(pedge != null);
        // da rivedere: viene aggiunta una sentinella parte && nel while
        while (pedge.CcwFace() != null && (pedge != psentry)) {
          pedge = pedge.m_pNext;
        }
      } else { // trova il primo spigolo connesso in senso orario
        pedge = m_pPrev;
        psentry = m_pPrev;
        Debug.Assert(pedge != null);
        while (pedge.CwFace() != null && (pedge != psentry)) {
          pedge = pedge.m_pPrev;
        }
      }
      return pedge;

    }

    internal bool BelongToEdge(double test_x, double test_y)
    {
      Point2d s1 = OrgCoord();
      Point2d s2 = DestCoord();

      Point3d testPt = new Point3d(test_x, test_y, 0);

      Line s = new Line(s1.X, s1.Y, 0, s2.X, s2.Y, 0);
      Point3d cp = s.ClosestPoint(testPt, true);

      double d = (cp - testPt).Length;
      double tol = 1e-7; // TODO

      return d<tol;
    }


    internal static bool RightOf(Point2d x, AM_Edge pedge)
    {
      return AM_Util.CCW(x, pedge.DestCoord(), pedge.OrgCoord());
    }

    internal static bool LeftOf(Point2d x, AM_Edge pedge)
    {
      return AM_Util.CCW(x, pedge.OrgCoord(), pedge.DestCoord());
    }

  }
}
