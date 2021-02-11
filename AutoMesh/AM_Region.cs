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
  class AM_Region // Vedi TGeoRegion
  {

    List<AM_Boundary> m_ArrayMeshBoundary = new List<AM_Boundary>();
    AM_Mesh2d m_Mesh2D = null;

    List<AM_RegionBoundary> m_Loops = new List<AM_RegionBoundary>();
    List<AM_RegionBoundary> m_InnerCurves = new List<AM_RegionBoundary>();

    List<AM_RegionVertex> m_ArrayInnerVertex = new List<AM_RegionVertex>();

    bool m_ForceEven = false;

    internal AM_Region()
    {

    }

    internal AM_Mesh2d Mesh2D
    {
      get {
        return m_Mesh2D;
      }
    }

    internal List<AM_RegionBoundary> Loops
    {
      get {
        return m_Loops;
      }
    }

    internal List<AM_RegionBoundary> InnerCurves
    {
      get {
        return m_InnerCurves;
      }
    }

    internal List<AM_RegionVertex> ArrayInnerVertex
    {
      get {
        return m_ArrayInnerVertex;
      }
    }

    internal bool AddCurve(Curve crv, double space, bool forceToOpened)
    {
      if (crv.IsClosed && space == 0) {
        AreaMassProperties area = AreaMassProperties.Compute(crv, AM_Util.FLT_EPSILON);

        // Pre estimates a common weight
        int numElements = 1000;
        double faceArea = area.Area / numElements;
        if (space == 0)
          space = Math.Sqrt(faceArea * 4 / Math.Sqrt(3));
      }

      if (crv is PolylineCurve) {
        return AddPolyline(crv as PolylineCurve, space, forceToOpened);
      } else if (crv is PolyCurve) {
        return AddPolycurve(crv as PolyCurve, space, forceToOpened);
      } else if (crv is ArcCurve) {
        var nurbs = crv.ToNurbsCurve();
        return AddNurbsCurve(nurbs, space, forceToOpened);
      } else if (crv is NurbsCurve) {
        return AddNurbsCurve(crv as NurbsCurve, space, forceToOpened);
      }

      return false;
    }

    bool AddPolyline(PolylineCurve polyline, double space, bool forceToOpened)
    {
      var b = AM_RegionBoundary.Create(polyline, space, forceToOpened);
      if (b != null) {
        m_Loops.Add(b);
        return true;
      }

      return false;
    }

    bool AddPolycurve(PolyCurve pc, double space, bool forceToOpened)
    {
      var b = AM_RegionBoundary.Create(pc, space, forceToOpened);
      if (b != null) {
        m_Loops.Add(b);
        return true;
      }

      return true;
    }

    bool AddNurbsCurve(NurbsCurve nurbs_crv, double space, bool forceToOpened)
    {
      var b = AM_RegionBoundary.Create(nurbs_crv, space, forceToOpened);
      if (b != null) {
        m_Loops.Add(b);
        return true;
      }

      return true;
    }

    internal void AddWeigthPoint(Point pt, double space)
    {
      var mv = new AM_RegionVertex(AM_Util.To2d(pt.Location), space);
      m_ArrayInnerVertex.Add(mv);
    }

    internal bool BuildMesh()
    {
      BuildVertexes();
      BuildBoundary();
      RecoverBoundary();
      SetBoundary();
      RefineMesh(true, null);

      return true;
    }

    private void BuildVertexes()
    {
      for (int i=0; i<m_Loops.Count; i++) {
        var loop = m_Loops[i];
        loop.BuildVertexes(m_ForceEven);
      }
    }

    bool BuildBoundary()
    {
      m_Mesh2D = new AM_Mesh2d();

      BoundingBox maxRect = new BoundingBox();
      if (!BuildMeshBoundary(ref maxRect))
        return false;

      // Inserisce i contorni
      m_Mesh2D.Init(maxRect);

      int numBoundary = m_ArrayMeshBoundary.Count;
      for (int i = 0; i < numBoundary; i++) {
        m_ArrayMeshBoundary[i].InsertIntoMesh(m_Mesh2D);
      }

      //// Inserisce i vertici isolati
      for (int i = 0; i < m_ArrayInnerVertex.Count; i++) {
        var vertex = m_ArrayInnerVertex[i];
        AM_Vertex pvertex = null;

        Point2d pt = vertex.Location;

        if (m_Mesh2D.InsertPoint(new Point2d(pt), vertex.Space, out pvertex)) {
          pvertex.Flag = 0x01;
          vertex.MeshVertex = pvertex;
        }
      }

      return true;
    }

    bool BuildMeshBoundary(ref BoundingBox maxRect)
    {
      maxRect = BoundingBox.Empty;

      for (int i = 0; i < m_Loops.Count; i++) {
        var pLoop = m_Loops[i];

        AM_Boundary boundary = new AM_Boundary();
        m_ArrayMeshBoundary.Add(boundary);

        boundary.SetLoopIndex(i);
        boundary.FlagHole = m_Loops[i].IsClosed;

        for (int j = 0; j < pLoop.NumSegments; j++) {
          var pVertex = pLoop.GetVertex(j);

          boundary.AddPoint(pVertex.Location, pVertex.Space, true);
          maxRect.Union( AM_Util.To3d(pVertex.Location));

          var arrayPoints = pLoop.GetGeneratedPoints(j);

          for (int k = 0; k < arrayPoints.Count; k++) {
            // Nota: questa riga serve per gestire l'orientamento
            // TODO: valuatare se necessaria
            //int n = (pEdge.m_Index & 0x01) ? (arrayPoint.size() - k - 1) : k;
            int n = k;
            var meshPt = arrayPoints[k];

            boundary.AddPoint(new Point2d(meshPt.X, meshPt.Y), meshPt.Z);
            maxRect.Union(new Point3d(meshPt.X, meshPt.Y, 0));
          }

        }
      }
      return true;
    }


    bool RecoverBoundary()
    {
      {
        int numBoundary = m_ArrayMeshBoundary.Count;

        List<Point3d> AddArray = new List<Point3d>();

        bool bflag = true;
        while (bflag) {
          bflag = false;

          for (int i = 0; i < numBoundary; i++) {
            AM_Boundary pBoundary = m_ArrayMeshBoundary[i];
            if (!pBoundary.FlagHole && i == numBoundary-1) {
              break;
            }

            for (int j = 0; j < pBoundary.GetNumGenVertex(); j++) {
              bflag |= pBoundary.RecoverGenEdge(m_Mesh2D, j, AddArray);
            }
          }
        }
      }

      return true;
    }

    bool SetBoundary()
    {
      {
        m_Mesh2D.DeleteInit();

        int numBoundary = m_ArrayMeshBoundary.Count;
        for (int i = 0; i < numBoundary; i++) {
          if (m_ArrayMeshBoundary[i].FlagHole) {
            m_Mesh2D.SetBoundary(m_ArrayMeshBoundary[i]);
          }

        }

        // Elimina eventuali spigoli isolati
        for (int i = 0; i < m_Mesh2D.ArrayWEdges.Count; i++) {
          AM_Edge pedge = m_Mesh2D.ArrayWEdges[i].Edge();
          if (pedge.CcwFace() == null && pedge.CwFace() == null)
            m_Mesh2D.DeleteEdge(pedge);
        }
      }

      return true;
    }

    bool RefineMesh(bool bRefine, AM_Mesh2d pSpcFtSurface)
    {
      {
        m_Mesh2D.FirstClassific();

        bool bUseSpace = true;

        if (bRefine) {
          if (pSpcFtSurface==null || !bUseSpace)
            m_Mesh2D.RefineMesh();
          else
            m_Mesh2D.RefineMesh(pSpcFtSurface);
        }

        //pMesh2D.RelaxMesh();
        m_Mesh2D.SmoothMesh();
        m_Mesh2D.CheckWEdge();

        m_Mesh2D.ResetZ();
      }

      return true;
    }


  }
}
