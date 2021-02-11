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
using System.Diagnostics;
using Rhino;
using Rhino.Commands;
using Rhino.DocObjects;
using Rhino.Geometry;
using Rhino.Input;
using Rhino.Input.Custom;

namespace AutoMesh
{
  public class AutoMeshCommand : Command
  {
    static bool m_PointPuffyness = false;
    static double m_PuffynessOffset = 3;

    static bool m_BorderPuffyness = false;
    static double m_BorderOffset = 3;

    public AutoMeshCommand()
    {
      // Rhino only creates one instance of each command class defined in a
      // plug-in, so it is safe to store a refence in a static property.
      Instance = this;
    }

    ///<summary>The only instance of this command.</summary>
    public static AutoMeshCommand Instance
    {
      get;
      private set;
    }

    ///<returns>The command name as it appears on the Rhino command line.</returns>
    public override string EnglishName
    {
      get {
        return "AutoMeshCommand";
      }
    }

    protected override Result RunCommand(RhinoDoc doc, RunMode mode)
    {
      var ptPuffynessOption = new OptionToggle(m_PointPuffyness, "False", "True");
      var ptOffsetOption = new OptionDouble(m_PuffynessOffset, 0.5, double.PositiveInfinity);
      var ptBorderPuffynessOption = new OptionToggle(m_BorderPuffyness, "False", "True");
      var borderOffsetOption = new OptionDouble(m_BorderOffset, 0.5, double.PositiveInfinity);

      var go = new GetOption();
      go.SetCommandPrompt("Get meshing properties");
      go.AcceptNothing(true);
      go.AcceptEnterWhenDone(true);

      int ptPuffyOptionIndex = go.AddOptionToggle("PointPuffyness", ref ptPuffynessOption);
      int offsetOptionIndex = go.AddOptionDouble("OffsetPtPuffy", ref ptOffsetOption);
      int borderPuffyOptionIndex = go.AddOptionToggle("BorderPuffyness", ref ptBorderPuffynessOption);
      int borderOffsetOptionIndex = go.AddOptionDouble("OffsetBorderPuffy", ref borderOffsetOption);

      go.Get();
      var result = go.Result();
      while (result != GetResult.Nothing) {
        if (result == GetResult.Cancel) {
          return Result.Cancel;
        }

        int optionIdx = go.OptionIndex();
        if (optionIdx == ptPuffyOptionIndex) {
          m_PointPuffyness = ptPuffynessOption.CurrentValue;
        } else if (optionIdx == offsetOptionIndex) {
          m_PuffynessOffset = ptOffsetOption.CurrentValue;
        } else if (optionIdx == borderPuffyOptionIndex) {
          m_BorderPuffyness = ptBorderPuffynessOption.CurrentValue;
        } else if (optionIdx == borderOffsetOptionIndex) {
          m_BorderOffset = borderOffsetOption.CurrentValue;
        }

        result = go.Get();
      }

      ObjRef[] rhObjects;
      var res = RhinoGet.GetMultipleObjects("Select planar curves and Weight points", false, Rhino.DocObjects.ObjectType.Curve | Rhino.DocObjects.ObjectType.Point, out rhObjects);

      if (res == Result.Success) {

        // 1. subdive in sets: Closed curves, Opened Curves, weight points;
        List<Curve> closed_crvs = new List<Curve>();
        List<Curve> opened_crvs = new List<Curve>();
        List<Point> weight_points = new List<Point>();

        bool puffyness = m_PointPuffyness;
        double offset = m_PuffynessOffset;

        bool border_puffyness = m_BorderPuffyness;
        double border_offset = m_BorderOffset;

        foreach (var ref_obj in rhObjects) {
          RhinoObject obj = ref_obj.Object();
          if (obj.Geometry is Curve) {
            var crv = obj.Geometry as Curve;
            if (crv.IsPlanar()) {
              if (crv.IsClosed) {
                closed_crvs.Add(crv);
              } else {
                opened_crvs.Add(crv);
              }
            }
          } else if (obj.Geometry is Point) {
            weight_points.Add(obj.Geometry as Point);
          }
        }

        double space = 1;

        // 2. Insert curves into mesh
        AM_Region region = null;

        Curve border_outer_crv = null;
        Curve offset_outer_crv = null;

        if (closed_crvs.Count > 0) {
          region = new AM_Region();

          for (int i = 0; i < closed_crvs.Count; i++) {
            var crv = closed_crvs[i];

            region.AddCurve(crv, space, false);

            AreaMassProperties area = AreaMassProperties.Compute(crv, AM_Util.FLT_EPSILON);
            if (area.Area > 0 && border_puffyness) {
              if (border_outer_crv == null) {
                border_outer_crv = crv;
              }

              var offset_Crvs = crv.Offset(Plane.WorldXY, -border_offset, AM_Util.FLT_EPSILON, CurveOffsetCornerStyle.None);

              foreach (var c in offset_Crvs) {
                c.Reverse();
                doc.Objects.AddCurve(c);
                offset_outer_crv = c;

                region.AddCurve(c, space, true);
              }
            }
          }
        } else {
          // TODO
          Debug.Assert(false);
          return Result.Failure;
        }

        for (int i = 0; i < weight_points.Count; i++) {
          var pt = weight_points[i];

          region.AddWeigthPoint(pt, space / 2);
          if (puffyness && offset > 0) {
            var circle = new ArcCurve(new Circle(pt.Location, offset));
            var nurbs = circle.ToNurbsCurve();
            region.AddCurve(nurbs, space / 2, true);
          }
        }

        for (int i = 0; i < opened_crvs.Count; i++) {
          var crv = opened_crvs[i];
          region.AddCurve(crv, space, false);

          if (puffyness && offset > 0) {
            var n = Vector3d.CrossProduct(crv.TangentAtStart, Vector3d.ZAxis);
            Curve[] offsetCrv = crv.Offset(crv.PointAtStart + offset * n, Vector3d.ZAxis, offset, AM_Util.FLT_EPSILON, CurveOffsetCornerStyle.Round);

            foreach (var c in offsetCrv) {
              doc.Objects.AddCurve(c);
              region.AddCurve(crv, space, false);
            }

            Curve[] offsetCrv2 = crv.Offset(crv.PointAtStart - offset * n, Vector3d.ZAxis, offset, AM_Util.FLT_EPSILON, CurveOffsetCornerStyle.Round);

            foreach (var c in offsetCrv2) {
              doc.Objects.AddCurve(c);
              region.AddCurve(crv, space, false);
            }

          }
        }

        // 3. Mesh della regione
        if (region != null) {
          if (region.BuildMesh()) {

            // Inserisce i punti del contorno
            foreach (var loop in region.Loops) {
              for (int i = 0; i < loop.NumSegments; i++) {
                var points = loop.GetGeneratedPoints(i);

                //if (points != null) {
                //  foreach (var p in points) {
                //    doc.Objects.AddPoint(new Point3d(p.X, p.Y, 0));
                //  }
                //}
              }
            }
          }

          // Trasforma in Mesh di Rhino
          var mesh = region.Mesh2D;

          if (mesh != null) {
            Mesh rhino_mesh = new Mesh();
            double t = 5;
            for (int i = 0; i < mesh.ArrayVertexes.Count; i++) {
              mesh.ArrayVertexes[i].Z = t;
            }

            // PostProcessa il puffyness
            if (puffyness) {

              for (int i = 0; i < region.ArrayInnerVertex.Count; i++) {
                var iv = region.ArrayInnerVertex[i];
                if (iv.MeshVertex != null) {
                  iv.MeshVertex.Z = (4 / 5d) * t;
                }

                // Ricerca i punti nell'intorno fino all'offset (molto grezza!)
                for (int j = 0; j < mesh.ArrayVertexes.Count; j++) {
                  var v = mesh.ArrayVertexes[j];
                  double d = (iv.MeshVertex.Coord - v.Coord).Length;
                  if (d < offset) {
                    double r = d / offset;

                    AM_Util.EInterpolation interpolation = AM_Util.EInterpolation.Parabolic;
                    v.Z = AM_Util.Interpolation(interpolation, iv.MeshVertex.Z, t, r);
                  }
                }
              }
            }

            // Individua i punti all'interno della zona di transizione
            List<int> transitionVts = new List<int>();

            if (border_puffyness && border_offset > 0) {
              // Individua i vertici di partenza e utilizza u flag di lavoro
              List<AM_Vertex> transitionStartVts = new List<AM_Vertex>();

              for (int i = 0; i < mesh.ArrayVertexes.Count; i++) {
                var v = mesh.ArrayVertexes[i];
                bool is_loop_vertex = (v.Flag & 0x1) > 0;
                v.Flag &= ~0x02;

                if (is_loop_vertex && BelongToBorder(v)) {
                  transitionStartVts.Add(v);
                }
              }

              // Si usa 0x04 come flag di lavoro
              for (int i = 0; i < mesh.ArrayWEdges.Count; i++) {
                var e = mesh.ArrayWEdges[i];
                e.Edge().Flag &= ~0x04;
                e.Edge().Symm().Flag &= ~0x04;
              }

              for (int i = 0; i < transitionStartVts.Count; i++) {
                var v = transitionStartVts[i];
                AddTransitionVertexes(v, transitionVts);
              }

              if (offset_outer_crv != null)
                foreach (var iv in transitionVts) {
                  var v = mesh.ArrayVertexes[iv];

                  double par;
                  if (offset_outer_crv.ClosestPoint(AM_Util.To3d(v.Coord), out par, 2 * border_offset)) {
                    Point3d cp = offset_outer_crv.PointAt(par);
                    double r = ((cp - AM_Util.To3d(v.Coord)).Length) / border_offset;
                    double z = AM_Util.Interpolation(AM_Util.EInterpolation.Parabolic, 0.8 * t, t, 1 - r);
                    v.Z = z;
                  }
                }
            }

            // Facce
            int totVtx = mesh.ArrayVertexes.Count;

            for (int iSide = 0; iSide < 2; iSide++) {

              for (int i = 0; i < mesh.ArrayVertexes.Count; i++) {
                var v = mesh.ArrayVertexes[i];
                Point3d pt = v.Coord3d;
                if (iSide == 1) {
                  pt.Z = 0;
                }

                rhino_mesh.Vertices.Add(pt);
              }

              for (int i = 0; i < mesh.ArrayFaces.Count; i++) {
                var f = mesh.ArrayFaces[i];

                int numEdges = f.NumEdges;
                if (numEdges == 3) {
                  int[] vtx = { f.Vertex(0).Index, f.Vertex(1).Index, f.Vertex(2).Index };
                  if (iSide == 1) {
                    vtx = new int[] { f.Vertex(0).Index + totVtx, f.Vertex(2).Index + totVtx, f.Vertex(1).Index + totVtx };
                  }
                  rhino_mesh.Faces.AddFace(vtx[0], vtx[1], vtx[2]);
                } else if (numEdges == 4) {
                  int[] vtx = { f.Vertex(0).Index, f.Vertex(1).Index, f.Vertex(2).Index, f.Vertex(3).Index };
                  if (iSide == 1) {
                    vtx = new int[] { f.Vertex(0).Index + totVtx, f.Vertex(3).Index + totVtx, f.Vertex(2).Index + totVtx, f.Vertex(1).Index + totVtx };
                  }

                  rhino_mesh.Faces.AddFace(vtx[0], vtx[1], vtx[2], vtx[3]);
                }
              }
            }

            for (int iEdge = 0; iEdge < mesh.ArrayWEdges.Count; iEdge++) {
              var edge = mesh.ArrayWEdges[iEdge].Edge();
              if (edge.CcwFace() == null || edge.CwFace() == null) {
                // E' uno spigolo di bordo
                int[] vtx = { edge.Destination().Index, edge.Origin().Index,
                              edge.Origin().Index  + totVtx, edge.Destination().Index + totVtx,
                            };
                rhino_mesh.Faces.AddFace(vtx[0], vtx[1], vtx[2], vtx[3]);
              }
            }

            rhino_mesh.Normals.ComputeNormals();
            rhino_mesh.Compact();
            if (doc.Objects.AddMesh(rhino_mesh) != Guid.Empty) {
              doc.Views.Redraw();
              return Rhino.Commands.Result.Success;
            }
          }

        }

        return Result.Success;
      }


      return Result.Cancel;
    }

    private void AddTransitionVertexes(AM_Vertex v, List<int> transitionVts)
    {
      if ((v.Flag & 0x02) == 0) {
        v.Flag |= 0x02;
        transitionVts.Add(v.Index);

        AM_Edge start_edge = v.Edge;
        AM_Edge edge = start_edge.Next;

        while (edge != start_edge) {
          if ( (edge.Flag & 0x04) == 0) {
            edge.Flag |= 0x04;
            var dest = edge.Destination();
            if ((dest.Flag & 0x01)==0) {
              AddTransitionVertexes(dest, transitionVts);
            }
          }
          edge = edge.Next;
        }
      }
    }

    bool BelongToBorder(AM_Vertex v)
    {
      AM_Edge start_edge = v.Edge;
      if (start_edge != null) {
        AM_Edge edge = start_edge.Next;
        while (edge != start_edge) {
          if (edge.CcwFace() == null || edge.CwFace() == null) {
            return true;
          }
          edge = edge.Next;
        }
      }

      return false;
    }
  }
}
