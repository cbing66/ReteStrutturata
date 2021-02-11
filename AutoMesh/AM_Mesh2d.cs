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
  class AM_Mesh2d
  {
    // --- Dati membro ---
    List<AM_Vertex> m_ArrayVertexes = new List<AM_Vertex>();    // vettore globale dei vertici
    List<AM_Coedge> m_ArrayWEdges = new List<AM_Coedge>();     // vettore globale delle strutture AM_Coedge
    List<AM_Face> m_ArrayFaces = new List<AM_Face>();      // vettore globale delle facce

    bool m_bFlagClassific;
    List<AM_Face> m_ArrayActFace = new List<AM_Face>();   // Array delle facce attive

    AM_Edge m_StartingEdge = null;

    AM_VertexTree m_pRangeSearch = new AM_VertexTree();

    AM_Mesh2d m_pSpaceFunction;

    // TODO: classe che raggruppa i dati
    static double m_Delta = 0.8;     //limite rapporto tra raggio teorico e effettivo
    static double m_RadCoef = 0.577;   //coeff. per il calcolo del raggio teorico
    static double m_SpaceCoef = 0.5; //coefficente riduttivo per il test di spaziatura

    static double m_Omega = 0.4;          // coefficente omega per lo smoothing
    static int m_NumSmoothing = 5;   // numero di ripetizioni dello smoothing

    internal AM_Mesh2d()
    {

    }

    internal List<AM_Vertex> ArrayVertexes
    {
      get {
        return m_ArrayVertexes;
      }
    }

    internal List<AM_Coedge> ArrayWEdges
    {
      get {
        return m_ArrayWEdges;
      }
    }

    internal List<AM_Face> ArrayFaces
    {
      get {
        return m_ArrayFaces;
      }
    }

    internal AM_VertexTree RangeSearch
    {
      get {
        return m_pRangeSearch;
      }
    }

    internal static double Delta
    {
      get {
        return m_Delta;
      }
    }

    internal static double RadCoef
    {
      get {
        return m_RadCoef;
      }
    }

    internal static double SpaceCoef
    {
      get {
        return m_SpaceCoef;
      }
    }


    internal static double Omega
    {
      get {
        return m_Omega;
      }
    }

    internal static int NumSmoothing
    {
      get {
        return m_NumSmoothing;
      }
    }


    internal void Init(BoundingBox rect)
    {
      Debug.Assert(m_ArrayFaces.Count==0);

      // si calcola un triangolo intorno alla massima
      // estensione dei punti; il triangolo costruito
      // è un triangolo isoscele avente la base poggiante
      // sulla base del rettangolo

      BoundingBox boundRect = rect;
      double w = boundRect.Diagonal.X/4;
      double h = boundRect.Diagonal.Y/4;

      boundRect.Inflate(w, h, 0);

      Point3d p1 = new Point3d(boundRect.Min.X -0.5 * boundRect.Diagonal.X, boundRect.Min.Y, 0);
      Point3d p2 = new Point3d(boundRect.Min.X + 1.5 * boundRect.Diagonal.X, boundRect.Min.Y, 0);
      Point3d p3 = new Point3d(boundRect.Min.X + 0.5 * boundRect.Diagonal.X, boundRect.Max.Y + boundRect.Diagonal.Y, 0);

      Init(p1, p2, p3);
    }

    void Init(Point3d a, Point3d b, Point3d c)
    {
      Debug.Assert(m_ArrayFaces.Count==0);

      AM_Face face = new AM_Face();
      Point3d[] coord = { a, b, c };
      AddFace(face, coord);
      m_StartingEdge = face[0];

    }

    void SwapPolygon(List<Point3d> array)
    {
      int sz = array.Count;
      for (int i = 0; i < sz / 2; i++) {
        Point3d tmp = array[i];
        array[i] = array[sz - i - 1];
        array[sz - i - 1] = tmp;
      }
    }

    internal void DeleteInit()
    {
      Point2d[] deletePoint = {m_ArrayVertexes[0].Coord,
                               m_ArrayVertexes[1].Coord,
                               m_ArrayVertexes[2].Coord
                              };

      // vengono cancellati i tre punti del triangolo esterno

      DeleteVertex(deletePoint[0], true);
      DeleteVertex(deletePoint[1], true);
      DeleteVertex(deletePoint[2], true);

    }

    // --- Attributi ---
    BoundingBox MaxRect()
    {
      BoundingBox rect = BoundingBox.Empty;

      for (int i = 0; i < m_ArrayVertexes.Count; i++) {
        AM_Vertex vertex = m_ArrayVertexes[i];
        rect.Union(new Point3d(vertex.Coord.X, vertex.Coord.Y, 0));
      }
      return rect;
    }

    // --- Gestione degli elementi (edge, face) ---
    int FindVertex(Point2d coord)
    {
      // Restituisce l'indice del vertice corrispondente coord.
      for (int i = 0; i < m_ArrayVertexes.Count; i++) {
        if (m_ArrayVertexes[i].IsEqual(coord, AM_Util.FLT_EPSILON)) {
          // restituisce l'indice trovato
          return i;
        }
      }

      // se la ricerca è fallita si aggiunge un nuovo vertice
      // all'array
      int nindex = m_ArrayVertexes.Count;
      m_ArrayVertexes.Add(new AM_Vertex(coord, 0, 0));
      return nindex;
    }

    AM_Edge FindEdge(AM_Vertex org, AM_Vertex dest)
    {
      AM_Edge pstart = org.Edge;
      if (pstart != null) {
        AM_Edge pnext = pstart;
        do {
          if (pnext.Destination() == dest)
            return pnext; // lo spigolo esiste già
          pnext = pnext.Next;
        } while (pnext != pstart);
      }
      return null;

    }

    internal int AddVertex(Point2d coord, double space, bool check = true)
    {
      // Questa routine è decisamente inefficiente;
      // è stata resa virtuale per essere adattata nei casi derivati

      if (check) {
        if (m_pRangeSearch != null) {
          // Ricerca su albero
          AM_Vertex vertex = m_pRangeSearch.Search(coord.X, coord.Y);
          if (vertex != null)
            return vertex.Index;
        } else {
          double tollerance = AM_Util.FLT_EPSILON;
          for (int i = m_ArrayVertexes.Count - 1; i >= 0; i--) {
            if (m_ArrayVertexes[i].Coord.EpsilonEquals(coord, tollerance)) {
              // Se il punto esiste già restituisce la sua posizione
              // nell'array globale dei vertici
              return i;
            }
          }
        }
      }

      // Il vertice non esiste ne viene creato uno nuovo
      AM_Vertex pvertex = new AM_Vertex(coord, 0, space);

      if (pvertex == null) {
        Debug.Assert(false);
        //throw -1;
      }

      int nindex = m_ArrayVertexes.Count;
      m_ArrayVertexes.Add(pvertex);
      pvertex.Index = nindex;

      return nindex;
    }

    internal bool DeleteVertex(Point2d coord, bool bdelete = false)
    {
      // Trova l'indice del vertice da eliminare

      double tollerance = AM_Util.FLT_EPSILON;
      int nindex;
      for (nindex = 0; nindex < m_ArrayVertexes.Count; nindex++) {
        if (m_ArrayVertexes[nindex].Coord.EpsilonEquals(coord, tollerance)) {
          // Se il punto esiste già restituisce la sua posizione
          // nell'array globale dei vertici
          break;
        }
      }

      if (nindex == m_ArrayVertexes.Count) {
        // il vertice non esiste
        return false;
      }

      AM_Vertex pvertex = m_ArrayVertexes[nindex];

      // se questa condizione non viene rispettata è evidente
      // che ci sono problemi di coerenza. Chiamare UpdateIndex eventualmente
      Debug.Assert(pvertex.Index == nindex);

      AM_Edge edge = pvertex.Edge;
      AM_Edge pendEdge = edge.Prev;

      if (edge == pendEdge) {
        // Lo spigolo è isolato
        Debug.Assert(edge.CcwFace() == null);
        Debug.Assert(edge.CcwFace() == null);
        DeleteEdge(edge);
      } else {
        // cancella gli spigoli collegati e le facce relative
        while (true) {
          AM_Edge pdeleteEdge = edge;
          edge = pdeleteEdge.Next;

          // SHOWEDGE(edge);
          // SHOWEDGE(pdeleteEdge);
          DeleteEdge(pdeleteEdge);

          if (edge == pendEdge) {
            DeleteEdge(edge);
            break;
          }
        }
      }

      DeleteVertex(pvertex, bdelete);

      return true;

    }

    bool DeleteVertex(AM_Vertex vertex, bool bdelete = false)
    {
      // Cancella un vertice dall'array globale dei vertici
      if (vertex == null || vertex.Index < 0) {
        Debug.Assert(false);
        return false;
      }

      int nindex = vertex.Index;
      int nlast = m_ArrayVertexes.Count - 1;

      // La cancellazione dall'array globale degli vertici avviene
      // spostando l'ultimo vertice dell'array al posto di quello da cancellare
      m_ArrayVertexes[nindex] = m_ArrayVertexes[nlast];
      m_ArrayVertexes[nindex].Index = nindex;
      m_ArrayVertexes.RemoveAt(nlast);

      if (nlast != nindex) {
        // è necessario aggiornare opportunamente l'anello del vertice
        // aggiornando il dato membro m_nVertex;
        AM_Edge pstartEdge = m_ArrayVertexes[nindex].Edge;
        AM_Edge edge = pstartEdge;

        if (pstartEdge == null) {
          Debug.Assert(false);
          return true;
        }

        do {
          edge.Vertex = m_ArrayVertexes[nindex];
          edge = edge.Next;
        } while (edge != pstartEdge);

        if (bdelete) {
          //delete vertex;
          vertex.Edge = null;
          vertex.Index = -1;
        } else {
          vertex.Edge = null;
          vertex.Index = -1;
        }
      }

      return true;

    }

    AM_Edge AddEdge(AM_Vertex org, AM_Vertex dest)
    {
      AM_Coedge pwEdge = new AM_Coedge(org, dest);

      if (pwEdge == null) {
        Debug.Assert(false);
        //throw -1;
      }

      pwEdge.Index = m_ArrayWEdges.Count;
      m_ArrayWEdges.Add(pwEdge);

      AM_Edge edge = pwEdge.Edge();
      //edge.m_pArrayVertex = &m_ArrayVertexes;
      //edge.Symm().m_pArrayVertex = &m_ArrayVertexes;
      return edge;
    }

    internal bool DeleteEdge(AM_Edge edge, bool bDelIsolatedVertex = false)
    {
      Debug.Assert(edge != null);
      // Cancella le facce collegate allo spigolo
      DeleteFace(edge.CwFace());
      DeleteFace(edge.CcwFace());

      AM_Coedge pwEdge = edge.WingedEdge;
      AM_Coedge pdeletingEdge = m_ArrayWEdges[pwEdge.Index];
      int nlast = m_ArrayWEdges.Count - 1;
      Debug.Assert(pdeletingEdge == pwEdge);

      // La cancellazione dall'array globale degli spigoli avviene
      // spostando l'ultimo spigolo dell'array al posto di quello da cancellare
      m_ArrayWEdges[pwEdge.Index] = m_ArrayWEdges[nlast];
      m_ArrayWEdges[pwEdge.Index].Index = pwEdge.Index;
      m_ArrayWEdges.RemoveAt(nlast);

      AM_Vertex pIsolated1 = null;
      AM_Vertex pIsolated2 = null;

      // ripristina le connessioni
      edge.Next.Prev = edge.Prev;
      edge.Prev.Next = edge.Next;
      if (edge.Next == edge && edge.Prev == edge) {
        // rimane un vertice isolato
        pIsolated1 = edge.Origin();
        pIsolated1.Edge = null;
      } else {
        edge.Origin().Edge = edge.Next;
      }

      // ripristina le connessioni del duale
      edge = edge.Symm();
      edge.Next.Prev = edge.Prev;
      edge.Prev.Next = edge.Next;
      if (edge.Next == edge && edge.Prev == edge) {
        // rimane un vertice isolato
        pIsolated2 = edge.Origin();
        pIsolated2.Edge = null;
      } else {
        edge.Origin().Edge = edge.Next;
      }

      //delete pwEdge;

      if (bDelIsolatedVertex) {
        if (pIsolated1 != null)
          DeleteVertex(pIsolated1, true);

        if (pIsolated2 != null)
          DeleteVertex(pIsolated2, true);
      }

      edge.Next = null;
      edge.Prev = null;

      return true;
    }

    bool AddFace(AM_Face face, Point3d [] Coord)
    {
      int numVertex = Coord.Length;
      int [] pnvertex = new int[numVertex];

      // localizza i vertici
      for (int i = 0; i < numVertex; i++) {
        pnvertex[i] = AddVertex(new Point2d(Coord[i]), 0);
      }

      bool bRet = AddFace(face, pnvertex);

      return bRet;
    }

    bool AddFace(AM_Face face, int [] nvertex)
    {
      int numVertex = nvertex.Length;
      Debug.Assert(face != null);
      int iLast = m_ArrayFaces.Count;
      face.Index = iLast;
      m_ArrayFaces.Add(face);

      // localizza gli spigoli (in senso antiorario)
      for (int i = 0; i < numVertex; i++) {
        int v0 = nvertex[i];
        int v1 = nvertex[(i + 1) % numVertex];

        face[i] = FindEdge(m_ArrayVertexes[v0], m_ArrayVertexes[v1]);

        // se lo spigolo non esiste viene aggiunto nell'array
        if (face[i] == null)
          face[i] = AddEdge(m_ArrayVertexes[v0], m_ArrayVertexes[v1]);
      }

      if (!face.SetConnection()) {
        return false;
      }
      return true;
    }

    bool DeleteFace(AM_Face face, bool bremoveEdge = false)
    {
      // face          faccia da rimuovere
      // bremoveEdge    flag indicante la rimozione forzata di spigoli isolati

      if (face == null) {
        return false;
      }

      // annulla i puntatori degli spigoli che definiscono la faccia
      for (int i = 0; i < face.NumEdges; i++) {
        face[i].Face = null;
      }

      AM_Face pdeletingFace = m_ArrayFaces[face.Index];
      int nlast = m_ArrayFaces.Count - 1;
      Debug.Assert(pdeletingFace == face);

      // La cancellazione dall'array globale degli spigoli avviene
      // spostando l'ultimo spigolo dell'array al posto di quello da cancellare
      m_ArrayFaces[face.Index] = m_ArrayFaces[nlast];
      m_ArrayFaces[face.Index].Index = face.Index;
      m_ArrayFaces.RemoveAt(nlast);

      if (bremoveEdge) {
        // rimuove gli spigoli isolati
        for (int i = 0; i < face.NumEdges; i++) {
          // se la faccia destra e sinistra dello spigolo
          // non esistono significa che lo spigolo è isolato
          if (face[i].CwFace() != null && face[i].CcwFace() != null) {
            DeleteEdge(face[i]);
          }
        }
      }

      //delete face;
      return true;
    }

    bool RecoverBoundary(AM_Boundary boundary)
    {
      // se viene inserito un punto per aggiustare la conformità
      // il flag baddFlag diventa true
      bool baddFlag = false;

      AM_Edge pbase = null;
      AM_Edge pprev = pbase = boundary.Vertex(0).Edge;

      for (int i = 1; (boundary.FlagHole ? i <= boundary.GetNumVertex()
                       : i < boundary.GetNumVertex()); i++) {
        // si controlla che tutti i vertici siano in sequenza
        while (true) {
          Point3d p1 = boundary[i - 1];
          Point3d p2 = boundary[i % (boundary.GetNumVertex())];

          if (pbase.DestCoord() == new Point2d(p2)) {
            // il vertice è in sequenza: si continua con il successivo
            break;
          } else {
            pbase = pbase.Next;

            if (pbase == pprev) {
              // il ciclo dell'anello si è chiuso senza trovare il vertice
              // successivo; è necessario inserire un vertice in mezzeria del
              // lato mancante

              baddFlag = true;  // si segnala l'aggiunta di un vertice

              Point3d mid = 0.5 * (p1 + p2);

              // si inserisce un vertice nel mezzo dello spigolo
              AM_Vertex pvertex = null;
              InsertPoint(new Point2d(mid), mid.Z, out pvertex);

              if (pvertex == null) {
                Debug.Assert(false);
                //throw 6;
              }

              boundary.InsertVertex(i, pvertex);

              // si ricomincia il controllo
              pbase = boundary.Vertex(i - 1).Edge;
              pprev = pbase;
            }
          }
        }
        pbase = boundary.Vertex(i%(boundary.GetNumVertex())).Edge;
        pprev = pbase;
      }

      return baddFlag;
    }

    internal bool SetBoundary(AM_Boundary boundary)
    {
      AM_Edge edge;

      int numVertex = boundary.GetNumVertex();

      // Viene marcato il flag di contorno
      for (int i = 0; i < boundary.GetNumVertex(); i++) {
        edge = boundary.Vertex(i).Edge;
        Point2d nextCoord = new Point2d(boundary[(i + 1) % numVertex]);

        // si trova lo spigolo del contorno
        while (!edge.DestCoord().EpsilonEquals(nextCoord, AM_Util.FLT_EPSILON)) {
          edge = edge.Next;
        }

        edge.Flag |= 0x02;
        edge.Symm().Flag |= 0x02;
      }

      // Trova lo spigolo iniziale
      AM_Edge pprevEdge = boundary.Vertex(numVertex - 1).Edge;
      while (!pprevEdge.DestCoord().EpsilonEquals(new Point2d(boundary[0]), 1e-8)) {
        pprevEdge = pprevEdge.Next;
      }
      pprevEdge = pprevEdge.Symm();   // viene usato come sentinella

      // controllo su tutti gli spigoli del contorno
      for (int i = 1; i <= boundary.GetNumVertex(); i++) {
        edge = boundary.Vertex(i - 1).Edge;

        Point2d nextCoord = new Point2d(boundary[i % numVertex]);

        // si trova lo spigolo del contorno
        while (edge.DestCoord() != nextCoord) {
          edge = edge.Next;
        }

        if (edge.CwFace() != null) {
          // se c'è una faccia a dx dello spigolo
          // ci sono spigoli da eliminare
          AM_Edge plookEdge = edge.Prev;
          while (plookEdge != pprevEdge) {
            AM_Edge pdeleteEdge = plookEdge;
            plookEdge = plookEdge.Prev;

            if ((pdeleteEdge.Flag & 0x02) == 0) {
              // Non è uno spigolo di bordo
              DeleteEdge(pdeleteEdge);
            }
          }
        }
        pprevEdge = edge.Symm();
      }

      return true;
    }

    internal bool InsertPoint(Point2d x, double space, out AM_Vertex pvertex)
    {
      pvertex = null;
      AM_Face face = null;

      // Localizza uno spigolo vicino
      AM_Edge edge = Locate(x);
      if (edge == null) {
        return false;
      }

      // Localizza il triangolo che contiene il punto x
      // e imposta 'm_pStartingEdge', primo spigolo del triangolo o del quadrilatero
      // che deve essere riconnesso al punto x
      if (AM_Edge.LeftOf(x, edge)) {
        face = (AM_Face)(edge.CcwFace());
        m_StartingEdge = edge.CcwEdge();
      } else {
        face = (AM_Face)(edge.CwFace());
        m_StartingEdge = edge.Symm().CcwEdge();
      }

      if (face == null) {
        return false;
      }

      // Verifica dell'eventuale esistenza del punto
      if (x == edge.OrgCoord()) {
        pvertex = edge.Origin();
        return false;
      }

      if (x == edge.DestCoord()) {
        pvertex = edge.Destination();
        return false;
      }

      Point2d[] v1 = { face.Vertex(0).Coord, face.Vertex(1).Coord, face.Vertex(2).Coord, };

      //isOnEdge = OnEdge(x, edge);
      AM_Edge pOnEdge = OnFaceEdge(x, face);

      if (pOnEdge != null) {
        m_StartingEdge = pOnEdge.CcwEdge();

        // il punto si trova su un contorno!
        AM_Face pCwFace = pOnEdge.CwFace();
        AM_Face pCcwFace = pOnEdge.CcwFace();

        if (pCwFace==null || pCcwFace==null)
          return false;
      }

      // Il punto è all'interno di un triangolo o su uno spigolo
      if (face.FaceType == AM_Face.EFaceType.FT_ACTIVE) {
        DeleteActiveFace(face);
      }

      DeleteFace(face);
      if (pOnEdge != null) {
        // Cancella lo spigolo su cui si appoggia e
        // conseguentemente anche l'altro spigolo

        AM_Face pCwFace = pOnEdge.CwFace();
        AM_Face pCcwFace = pOnEdge.CcwFace();

        if (pCwFace != null && pCwFace.FaceType == AM_Face.EFaceType.FT_ACTIVE)
          DeleteActiveFace(pCwFace);
        if (pCcwFace != null && pCcwFace.FaceType == AM_Face.EFaceType.FT_ACTIVE)
          DeleteActiveFace(pCcwFace);

        DeleteEdge(pOnEdge);
      }

      // Inserisce il nuovo vertice nell'array globale
      pvertex = new AM_Vertex(x, 0, space);
      if (pvertex == null) {
        Debug.Assert(false);
        //throw -1;
      }


      int m_nVertex = m_ArrayVertexes.Count;
      pvertex.Index = m_ArrayVertexes.Count;
      m_ArrayVertexes.Add(pvertex);

      // Inserisce i nuovi triangoli (facce)
      edge = m_StartingEdge.CcwEdge();
      int numEdge = (pOnEdge != null? 4 : 3);
      for (int ne = 0; ne < numEdge; ne++) {
        AM_Face new_face = new AM_Face();
        if (new_face == null) {
          Debug.Assert(false);
          //throw -1;
        }

        AM_Edge actEdge = edge;
        edge = edge.CcwEdge();
        int [] nCoord = { m_nVertex, actEdge.Vertex.Index, actEdge.DestVertex().Index };
        AddFace(new_face, nCoord);

        if (m_bFlagClassific) {
          new_face.SetTriangleParameter(m_pSpaceFunction);
          Classific(new_face);
        }
      }

      // Esamina gli spigoli per assicurare che la condizione di
      // Delaunay sia soddisfatta
      edge = m_StartingEdge;
      m_StartingEdge = m_StartingEdge.CcwEdge();
      do {
        //TRACE_EDGE(edge);
        AM_Edge t = edge.Prev;
        if (edge.CwFace() != null && AM_Edge.RightOf(t.DestCoord(), edge)
            && AM_Util.InCircle(edge.OrgCoord(), t.DestCoord(), edge.DestCoord(), x)) {
          //TRACE0("Faccia swap:  ");
          //TRACE_EDGE(edge);
          Swap(edge);
          edge = edge.Prev;
        } else if (edge.Next == m_StartingEdge) {
          // Non ci sono più spigoli
          break;
        } else {
          // Recupera un altro spigolo sospetto
          edge = edge.Next.CwEdge();
        }
      } while (true);

      return true;
    }

    void Swap(AM_Edge edge)
    {
      AM_Face face1 = (AM_Face)edge.CcwFace();
      AM_Face face2 = (AM_Face)edge.CwFace();

      if (face1==null || face2 == null) {
        if (face1 != null && face1.FaceType == AM_Face.EFaceType.FT_ACTIVE)
          DeleteActiveFace(face1);
        if (face2 != null && face2.FaceType == AM_Face.EFaceType.FT_ACTIVE)
          DeleteActiveFace(face2);
        return;
      }

      Debug.Assert(edge.CwFace() != null);
      Debug.Assert(edge.CcwFace() != null);

      if (face1.FaceType == AM_Face.EFaceType.FT_ACTIVE)
        DeleteActiveFace(face1);

      if (face2.FaceType == AM_Face.EFaceType.FT_ACTIVE)
        DeleteActiveFace(face2);

      AM_Face.SwapEdge(edge);

      if (m_bFlagClassific) {
        face1.SetTriangleParameter(m_pSpaceFunction);
        face2.SetTriangleParameter(m_pSpaceFunction);
        Classific(face1);
        Classific(face2);
      }
    }

    AM_Edge Locate(Point2d x)
    {
      AM_Edge e = m_StartingEdge;
      int actualEdge = 0;
      int numEdge = m_ArrayWEdges.Count;

      while (actualEdge++ <= numEdge) {
        if (x == e.OrgCoord() || x == e.DestCoord())
          return e;
        else if (AM_Edge.RightOf(x, e))
          e = e.Symm();
        else if (!AM_Edge.RightOf(x, e.Next))
          e = e.Next;
        else if (!AM_Edge.RightOf(x, e.CcwEdge().Symm()))
          e = e.CcwEdge().Symm();
        else
          return e;
      }
      return null;
    }

    AM_Edge OnFaceEdge(Point2d x, AM_Face face)
    {
      for (int i = 0; i < face.NumEdges; i++) {
        if (OnEdge(x, face[i]))
          return face[i];
      }
      return null;
    }

    bool OnEdge(Point2d p, AM_Edge edge)
    {
      double t1, t2, t3;
      double EPS = 1e-6;

      Point2d org = edge.OrgCoord();
      t1 = (p - org).Length;
      t2 = (p - edge.DestCoord()).Length;
      if (t1 < EPS || t2 < EPS)
        return true;

      Vector2d vector = org -edge.DestCoord();
      t3 = vector.Length;
      if (t1 > t3 || t2 > t3)
        return false;

      double a = vector.Y / t3;
      double b = -vector.X / t3;
      double c = -(a * org.X + b * org.Y);

      return (Math.Abs((a * p.X + b * p.Y + c)) < EPS);
    }

    internal bool GetCoordZ(Point2d p, ref double Z)
    {
      // Localizza uno spigolo vicino
      AM_Edge edge = Locate(p);
      if (edge == null) {
        return false;
      }

      AM_Face face = null;
      if (AM_Edge.LeftOf(p, edge))
        face = edge.CcwFace();
      else
        face = edge.CwFace();

      Z = 0;
      if (face == null)
        return false;

      Point2d p0 = face.Vertex(0).Coord;
      Point2d p1 = face.Vertex(1).Coord;
      Point2d p2 = face.Vertex(2).Coord;

      double det = AM_Util.TriArea(p0, p1, p2);
      Debug.Assert(det != 00);
      double alfa = AM_Util.TriArea(p0, p, p2) / det;
      double beta = AM_Util.TriArea(p0, p1, p) / det;

      double f0 = face.Vertex(0).Z;
      double f1 = face.Vertex(1).Z;
      double f2 = face.Vertex(2).Z;

      Z = f0 + alfa * (f1 - f0) + beta * (f2 - f0);

      return true;
    }

    internal void FirstClassific()
    {
      m_bFlagClassific = true;
      AM_Face face = null;

      for (int i = 0; i < m_ArrayFaces.Count; i++) {
        face = m_ArrayFaces[i];
        face.SetTriangleParameter(m_pSpaceFunction);
        face.FaceType = AM_Face.EFaceType.FT_NONE;
      }

      for (int i = 0; i < m_ArrayFaces.Count; i++) {
        face = (AM_Face)m_ArrayFaces[i];
        Classific(face);
      }
    }

    void Classific(AM_Face face)
    {
      if (face.FaceType == AM_Face.EFaceType.FT_ACCEPTED) {
        for (int i = 0; i < 3; i++) {
          AM_Face tmp = face.GetNearTriangle(i);
          if (!(tmp == null)) {
            if (tmp.FaceType != AM_Face.EFaceType.FT_ACCEPTED) {
              if (tmp.FaceType == AM_Face.EFaceType.FT_NONE) {
                AddActiveFace(tmp);
              } else if (tmp.FaceType == AM_Face.EFaceType.FT_WAITING) {
                AddActiveFace(tmp);
              }
            }
          }
        }
      } else {
        if (face.FaceType == AM_Face.EFaceType.FT_NONE) {
          for (int i = 0; i < 3; i++) {
            AM_Face tmp = face.GetNearTriangle(i);
            if (tmp == null || tmp.FaceType == AM_Face.EFaceType.FT_ACCEPTED) {
              AddActiveFace(face);
              break;
            } else
              face.FaceType = AM_Face.EFaceType.FT_WAITING;
          }
        }
      }

    }

    bool IsBoundaryVertex(AM_Vertex pvertex)
    {
      AM_Edge start = pvertex.Edge;
      if (start == null) {
        Debug.Assert(false);
        return false; // Indefinito
      }

      AM_Edge edge = start;
      do {
        if (edge.CwFace()==null || edge.CcwFace() == null)
          return true;

        edge = edge.Next;
      } while (start != edge);

      return false;
    }

    AM_Face FindMaxActive()
    {
      if (m_ArrayActFace.Count == 0)
        return null;

      AM_Face maxActive = ((AM_Face)m_ArrayActFace[0]);
      Debug.Assert(maxActive != null);
      double radMax = maxActive.CircumRadius;

      for (int i = 1; i < m_ArrayActFace.Count; i++) {
        AM_Face face = m_ArrayActFace[i];
        if (face.CircumRadius > radMax) {
          radMax = face.CircumRadius;
          maxActive = face;
        }
      }
      return maxActive;
    }

    internal bool RefineMesh(AM_Mesh2d pSpcFtSurface = null)
    {
      m_pSpaceFunction = pSpcFtSurface;
      while (m_ArrayActFace.Count != 0) {
        RefineMeshStep();
      }

      return true;
    }

    bool RefineMeshStep()
    {
      AM_Face face = null;

      Point2d innerPoint;

      AM_Face triangleContaining = null;
      face = FindMaxActive();

      if (face == null)
        return false;

      m_StartingEdge = face.GetStartingEdge();
      innerPoint = face.InsertPoint(m_pSpaceFunction);

      //localizzo il triangolo che contiene il punto per fare il test di spaziatura.
      AM_Edge e = Locate(innerPoint);
      triangleContaining = e != null? (AM_Edge.RightOf(innerPoint, e) ? e.CwFace() : e.CcwFace())
                           : null;

      if (triangleContaining == null)
        DeleteActiveFace(face);
      else {
        //innerPoint.Z = triangleContaining.SpaceFunction(innerPoint, m_pSpaceFunction);
        double s = triangleContaining.SpaceFunction(innerPoint, m_pSpaceFunction);
        if (triangleContaining.SpacingTest(innerPoint, m_pSpaceFunction)) {
          AM_Vertex vertex;
          if (!InsertPoint(innerPoint, s, out vertex))
            DeleteActiveFace(face);
        } else
          DeleteActiveFace(face);
      }

      return true;
    }

    void RelaxMesh()
    {
      // Esegue la procedura di "Mesh Relaxation"
      bool bPrev = m_bFlagClassific;
      m_bFlagClassific = false;
      for (int DiffDegree = 3; DiffDegree >= 2; DiffDegree--) {
        for (int i = 0; i < m_ArrayWEdges.Count; i++) {
          AM_Edge edge = m_ArrayWEdges[i].Edge();
          if (edge.CcwFace() != null && edge.CwFace() != null) {
            AM_Vertex [] Vertex = {edge.Origin(),
                                   edge.Destination(),
                                   edge.Next.Destination(),
                                   edge.Prev.Destination(),
                                  };
            double [] degree = { 0, 0, 0, 0 };
            for (int j = 0; j < degree.Length; j++)
              degree[j] = Vertex[j].Degree();

            double R = 0;
            for (int j = 0; j < 4; j++)
              R += (6 - degree[j]) * (6 - degree[j]);

            // aggiorna il grado con l'ipotesi do swap
            degree[0] -= 1;
            degree[1] -= 1;
            degree[2] += 1;
            degree[3] += 1;

            double R1 = 0;
            for (int j = 0; j < 4; j++)
              R1 += (6 - degree[j]) * (6 - degree[j]);

            if (R - R1 >= DiffDegree) {
              Swap(edge);
            }
          }
        }
      }
      m_bFlagClassific = bPrev;
    }

    internal double SmoothMesh()
    {
      double eps = 0;

      for (int n = 0; n < m_NumSmoothing; n++) { //ripeto Num volte lo smoothing
        eps = 0;
        for (int i = 0; i < m_ArrayVertexes.Count; i++) {
          // ciclo per tutti i punti interni al dominio
          AM_Vertex vertex = m_ArrayVertexes[i];

          // Versione corretta (AC 16-01-03)
          // Algoritmo di Optimal Smoothing (Borouchaki-George IJNME vol.40)
          if (vertex.Flag == 0) { // E' un punto smoothabile
            Point2d p0 = vertex.Coord;
            Point2d center = Point2d.Unset;
            int degree = (int)vertex.Degree(true);
            if (degree < 2)
              continue;

            AM_Edge start = vertex.Edge;
            AM_Edge nextEdge = start;

            Point2d newCoord = Point2d.Unset;
            double oldQuality = double.MaxValue;

            // Valuta la qualità dei triangoli prima dello spostamento
            // e calcola il nuovo centro
            for (int j = 0; j < degree; j++) {
              Debug.Assert(nextEdge.CcwFace() != null);
              Point2d p1 = nextEdge.DestCoord();
              Point2d p2 = nextEdge.Next.DestCoord();

              // Punto del teorico triangolo equilatero di p1-p2
              Vector2d v = p2-p1;
              Point2d mp = 0.5 * (p1 + p2);
              Point2d np = mp + Math.Sqrt(3d) * v.Length * AM_Util.NormalVersor(v);

              newCoord += np;

              double inRadius = 0;
              double circumRadius = 0;

              AM_Util.CircumCircle(p0, p1, p2, ref center, ref circumRadius);
              AM_Util.InCircle(p0, p1, p2, ref center, ref inRadius);
              double sgnArea = AM_Util.TriArea(p0, p1, p2) > 0 ? 1 : -1;

              double quality = sgnArea * inRadius / circumRadius;
              oldQuality = Math.Min(oldQuality, quality);

              nextEdge = nextEdge.Next;
            }
            Debug.Assert(nextEdge == start);

            newCoord.X /= degree;
            newCoord.Y /= degree;

            // Controlla l'accettabilità del nuovo centro
            double newQuality = double.MaxValue;

            for (int j=0; j<degree; j++) {
              Debug.Assert(nextEdge.CcwFace() != null);
              Point2d p1 = nextEdge.DestCoord();
              Point2d p2 = nextEdge.Next.DestCoord();

              double inRadius = 0;
              double circumRadius = 0;
              AM_Util.CircumCircle(newCoord, p1, p2, ref center, ref circumRadius);
              AM_Util.InCircle(newCoord, p1, p2, ref center, ref inRadius);
              double sgnArea = AM_Util.TriArea(newCoord, p1, p2)>0? 1 : -1;

              double quality = sgnArea * inRadius / circumRadius;
              newQuality = Math.Min(quality, newQuality);

              nextEdge = nextEdge.Next;
            }

            Debug.Assert(nextEdge == start);

            if (newQuality > 0 && newQuality>oldQuality) {
              // La qualità viene migliorata, il vertice viene spostato
              eps += (newCoord - p0).Length;
              vertex.Coord = newCoord;
            }
          }
        }
      }

      return eps;
    }

    internal void ResetZ()
    {
      for (int i = 0; i < m_ArrayVertexes.Count; i++) {
        m_ArrayVertexes[i].Z = 0;
      }
    }

    void AddActiveFace(AM_Face face)
    {
      Debug.Assert(face != null);
      if (face.FaceType != AM_Face.EFaceType.FT_ACTIVE) {
        face.FaceType = AM_Face.EFaceType.FT_ACTIVE;
        face.ActiveIndex = m_ArrayActFace.Count;
        m_ArrayActFace.Add(face);
      }
    }

    void DeleteActiveFace(AM_Face face)
    {
      if (face == null)
        return;

      int nlast = m_ArrayActFace.Count - 1;
      AM_Face lastFace = m_ArrayActFace[nlast];

      m_ArrayActFace[face.ActiveIndex] = lastFace;
      lastFace.ActiveIndex = face.ActiveIndex;
      m_ArrayActFace.RemoveAt(nlast);

      face.ActiveIndex = -1;
      face.FaceType = AM_Face.EFaceType.FT_NONE;
    }

    internal bool CheckWEdge()
    {
      for (int i = 0; i < m_ArrayWEdges.Count; i++) {
        AM_Edge edge = m_ArrayWEdges[i].Edge();
        Debug.Assert(edge.Origin() != edge.Destination());
      }

      for (int i = 0; i < m_ArrayFaces.Count; i++) {
        AM_Face edge = m_ArrayFaces[i];
        double area = AM_Util.TriArea(edge[0].OrgCoord(),
                                      edge[1].OrgCoord(),
                                      edge[2].OrgCoord());
        Debug.Assert(area > 0);
      }

      return true;
    }

    bool ExportMesh(string filename)
    {
      Debug.Assert(false); // TODO
      /*
      ofstream os(filename);

      char sep[] = "/ ______________________________________________________________________________";
      // Esporta la mesh in formato Straus7
      os << sep << endl;
      os << "/ STRAND 7.0 DATA EXCHANGE FILE" << endl << endl;

      os << sep << endl;
      os << "/ NODE COORDINATES" << endl << endl;

      for (int i = 0; i < SIZE_INT(m_ArrayVertexes); i++) {
        AM_Vertex pV = m_ArrayVertexes[i];
        CString format;
        format.Format("  Node %15d  %15.12e %15.12e %15.12e ", i + 1,
                      pV.m_Coord.X, pV.m_Coord.Y, pV.m_Coord.Z);

        os << format << endl;
      }


      os << endl << sep << endl;
      os << "/ PLATE ELEMENTS" << endl << endl;

      for (int i = 0; i < SIZE_INT(m_ArrayFaces); i++) {
        AM_Face face = m_ArrayFaces[i];

        CString format;
        if (face.GetNumEdge() == 3) {
          format.Format("  Tri3 %15d  1  1 %d %d %d", face.m_Index + 1,
                        (*face)[0].m_nVertex + 1,
                        (*face)[1].m_nVertex + 1,
                        (*face)[2].m_nVertex + 1);
        } else if (face.GetNumEdge() == 4) {
          format.Format("  Quad4 %15d  1  1 %d %d %d %d", face.m_Index + 1,
                        (*face)[0].m_nVertex + 1,
                        (*face)[1].m_nVertex + 1,
                        (*face)[2].m_nVertex + 1,
                        (*face)[3].m_nVertex + 1);
        } else
          Debug.Assert(false);

        os << format << endl;
      }
      os << endl;
      */
      return true;
    }
  }
}
