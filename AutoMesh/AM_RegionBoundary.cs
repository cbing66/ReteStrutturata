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
  class AM_RegionBoundary
  {
    List<Curve> m_Curves = new List<Curve>();
    List<AM_RegionVertex> m_Vertexes = new List<AM_RegionVertex>();
    List<Point4d> [] m_GenPoint = null;
    bool m_IsClosed = true;

    internal AM_RegionBoundary(bool isClosed)
    {
      m_IsClosed = isClosed;
    }

    internal int NumVertexes
    {
      get {
        return m_Vertexes.Count;
      }
    }

    internal int NumSegments
    {
      get {
        return m_IsClosed? m_Vertexes.Count : (m_Vertexes.Count-1);
      }
    }

    internal Point2d GetPoint(int i)
    {
      return m_Vertexes[i].Location;
    }

    internal Curve GetCurve(int i)
    {
      return m_Curves[i];
    }

    internal AM_RegionVertex GetVertex(int i)
    {
      return m_Vertexes[i];
    }

    internal List<Point4d> GetGeneratedPoints(int i)
    {
      return m_GenPoint[i];
    }

    internal bool IsClosed // TODO
    {
      get {
        return m_IsClosed;
      }
    }

    internal static AM_RegionBoundary Create(PolylineCurve polyline, double space, bool forceToOpened)
    {
      bool is_closed = polyline.IsClosed;
      bool boundary_closed = is_closed;
      if (forceToOpened)
        boundary_closed = false;

      AM_RegionBoundary b = new AM_RegionBoundary(boundary_closed);

      for (int iSegm = 0; iSegm < polyline.SpanCount; iSegm++) {
        Point3d p0 = polyline.Point(iSegm);
        Point3d p1 = polyline.Point((iSegm + 1) % polyline.SpanCount);
        if (!is_closed && iSegm == polyline.SpanCount-1) {
          p1 = polyline.Point(iSegm + 1);
        }

        b.m_Vertexes.Add(new AM_RegionVertex(new Point2d(p0.X, p0.Y), space));
        b.m_Curves.Add(new LineCurve(p0, p1));

        if (!is_closed && iSegm == polyline.SpanCount-1) {
          b.m_Vertexes.Add(new AM_RegionVertex(new Point2d(p1.X, p1.Y), space));
        } else if (is_closed && forceToOpened && iSegm == polyline.SpanCount - 1) {
          b.m_Vertexes.Add(new AM_RegionVertex(new Point2d(p1.X, p1.Y), space));
        }
      }

      b.m_GenPoint = new List<Point4d>[polyline.SpanCount];

      return b;
    }

    internal static AM_RegionBoundary Create(PolyCurve pc, double space, bool forceToOpened)
    {
      bool is_closed = pc.IsClosed;
      bool boundary_closed = is_closed;
      if (forceToOpened)
        boundary_closed = false;

      AM_RegionBoundary b = new AM_RegionBoundary(boundary_closed);


      for (int iCrv = 0; iCrv < pc.SegmentCount; iCrv++) {
        var crv = pc.SegmentCurve(iCrv);

        Point3d p0 = crv.PointAtStart;

        b.m_Vertexes.Add(new AM_RegionVertex(new Point2d(p0.X, p0.Y), space));
        b.m_Curves.Add(crv);

        if (!is_closed && iCrv == pc.SegmentCount - 1) {
          Point3d p1 = crv.PointAtEnd;
          b.m_Vertexes.Add(new AM_RegionVertex(new Point2d(p1.X, p1.Y), space));
        } else if (is_closed && forceToOpened && iCrv == pc.SegmentCount - 1) {
          Point3d p1 = crv.PointAtEnd;
          b.m_Vertexes.Add(new AM_RegionVertex(new Point2d(p1.X, p1.Y), space));
        }
      }

      b.m_GenPoint = new List<Point4d>[b.NumVertexes];

      return b;
    }


    internal static AM_RegionBoundary Create(NurbsCurve crv, double space, bool forceToOpened)
    {
      bool is_closed = crv.IsClosed;
      bool boundary_closed = is_closed;
      if (forceToOpened)
        boundary_closed = false;

      AM_RegionBoundary b = new AM_RegionBoundary(boundary_closed);

      Point3d p0 = crv.PointAtStart;
      Point3d p1 = crv.PointAtEnd;

      b.m_Vertexes.Add(new AM_RegionVertex(new Point2d(p0.X, p0.Y), space));
      b.m_Curves.Add(crv);

      if (!is_closed) {
        b.m_Vertexes.Add(new AM_RegionVertex(new Point2d(p1.X, p1.Y), space));
      } else if (is_closed && forceToOpened) {
        b.m_Vertexes.Add(new AM_RegionVertex(new Point2d(p1.X, p1.Y), space));
      }

      b.m_GenPoint = new List<Point4d>[b.NumVertexes];

      return b;
    }

    internal bool BuildVertexes(bool bForceEven)
    {
      int numSegments = NumSegments;

      for (int i = 0; i < numSegments; i++) {
        var crv = m_Curves[i];
        var pV1 = m_Vertexes[i];
        var pV2 = m_Vertexes[(i + 1) % m_Vertexes.Count];

        if (!IsClosed && i == numSegments - 1) {
          pV2 = m_Vertexes[i + 1];
        }

        double space1 = pV1.Space;
        double space2 = pV2.Space;

        Point2d org = pV1.Location;
        Point2d dest = pV2.Location;

        Point2d start = AM_Util.To2d(crv.PointAtStart);
        Point2d end = AM_Util.To2d(crv.PointAtEnd);

        Debug.Assert(org.EpsilonEquals(start, AM_Util.FLT_EPSILON));
        Debug.Assert(dest.EpsilonEquals(end, AM_Util.FLT_EPSILON));

        m_GenPoint[i] = new List<Point4d>();

        if (!BuildPointFromSpace(bForceEven, org, dest, crv, space1, space2, m_GenPoint[i]))
        { }

      }

      return true;
    }

    bool BuildPointFromSpace(bool bForceEven, Point2d org, Point2d dest, Curve crv,
                             double space1, double space2, List<Point4d> GenPoint)
    {
      GenPoint.Clear();

      double len = crv.GetLength();
      var domain = crv.Domain;

      double step = (space1 + space2) / 2;
      double dStep = space2 - space1;

      if (dStep>0) {
        double h1 = space1;
        double h2 = space2;

        Debug.Assert(h1 != 0 && h2 != 0); // la spaziatura non deve essere nulla

        double r = (len - h1) / (len - h2);
        double k = Math.Log(h2 / h1) / Math.Log(r);
        double kk = 0;
        int numJoint = AM_Util.IntNear(k);
        if (numJoint != 0) {
          kk = Math.Log(r * h2 / h1) / Math.Log(r) / (numJoint + 1);
        } else
          return false;

        Debug.Assert(!bForceEven);

        for (int j = 0; j < numJoint; j++) {
          double partialLen = h1 * (Math.Pow(r, kk * (j + 1)) - 1) / (r - 1);

          double percent = partialLen / len;
          double space = h1 + (h2 - h1) * percent;

          double t = percent;
          Debug.Assert(t > 0 && t < 1);

          double par = (percent * domain.Length);
          Point2d newCoord = AM_Util.To2d(crv.PointAt(par));

          GenPoint.Add(new Point4d(newCoord.X, newCoord.Y, space, t));
        }
      } else {
        int n = (int)(len / step);

        if (bForceEven) {
          n *= 2;
          if (n == 0)
            n = 2;
        }

        if (n != 0) {
          for (int k = 1; k < n; k++) {
            double t = (double)(k) / (double)(n);
            double space = len / n;

            double par = (t * domain.Length);
            Point2d newCoord = AM_Util.To2d(crv.PointAt(domain.Min + par));

            GenPoint.Add(new Point4d(newCoord.X, newCoord.Y, space, t));
          }
        } else {
          return false;
        }
      }

      return true;
    }
  }

}

