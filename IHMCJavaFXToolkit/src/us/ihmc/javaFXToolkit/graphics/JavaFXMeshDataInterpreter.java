package us.ihmc.javaFXToolkit.graphics;

import java.util.ArrayList;
import java.util.LinkedHashMap;
import java.util.LinkedHashSet;
import java.util.Map;
import java.util.Set;

import javax.vecmath.Point3f;
import javax.vecmath.TexCoord2f;
import javax.vecmath.Tuple2f;
import javax.vecmath.Tuple3f;
import javax.vecmath.Vector3f;

import gnu.trove.list.array.TIntArrayList;
import javafx.scene.shape.TriangleMesh;
import javafx.scene.shape.VertexFormat;
import us.ihmc.graphics3DAdapter.graphics.MeshDataHolder;

public class JavaFXMeshDataInterpreter
{
   public static TriangleMesh interpretMeshData(MeshDataHolder meshData)
   {
      if (meshData == null)
         return null;

      Point3f[] vertices = meshData.getVertices();
      TexCoord2f[] texturePoints = meshData.getTexturePoints();
      int[] polygonIndices = meshData.getPolygonIndices();
      int[] pointsPerPolygonCount = meshData.getPolygonStripCounts();
      Vector3f[] polygonNormals = meshData.getPolygonNormals();

      TIntArrayList facesVertexOnlyIndices = new TIntArrayList();
      TIntArrayList facesIndices = new TIntArrayList();

      int polygonIndicesStart = 0;
      for (int index = 0; index < pointsPerPolygonCount.length; index++)
      {
         int pointsForThisPolygon = pointsPerPolygonCount[index];
         int[] polygon = new int[pointsForThisPolygon];

         for (int i = 0; i < pointsForThisPolygon; i++)
         {
            polygon[i] = polygonIndices[polygonIndicesStart + i];
         }

         int[] splitIntoTriangles = splitPolygonIntoTriangles(polygon);

         for (int i : splitIntoTriangles)
         {
            facesVertexOnlyIndices.add(i);
            facesIndices.add(i); // vertex index
            if (polygonNormals != null)
               facesIndices.add(index); // normal index
            else
               facesIndices.add(i); // normal index
               
            facesIndices.add(i); // texture index
         }

         polygonIndicesStart += pointsForThisPolygon;
      }

      int[] indices = facesIndices.toArray();

      TriangleMesh triangleMesh = new TriangleMesh(VertexFormat.POINT_NORMAL_TEXCOORD);
      triangleMesh.getPoints().addAll(convertToFloatArray(vertices));
      triangleMesh.getTexCoords().addAll(convertToFloatArray(texturePoints));
      triangleMesh.getFaces().addAll(indices);
      triangleMesh.getFaceSmoothingGroups().addAll(new int[indices.length / triangleMesh.getFaceElementSize()]);
      if (polygonNormals == null)
      {
         float[] normals = findNormalsPerVertex(facesVertexOnlyIndices.toArray(), vertices);
         triangleMesh.getNormals().addAll(normals);
      }
      else
      {
         triangleMesh.getNormals().addAll(convertToFloatArray(polygonNormals));
      }

      return triangleMesh;
   }

   private static float[] findNormalsPerVertex(int[] indices, Point3f[] vertices)
   {
      Map<Integer, Set<Integer>> participatingFacesPerVertex = new LinkedHashMap<Integer, Set<Integer>>();

      Set<Integer> vertexFacesSet;
      for (int i = 0; i < indices.length; i++)
      {
         if (participatingFacesPerVertex.get(indices[i]) == null)
         {
            vertexFacesSet = new LinkedHashSet<Integer>();
            participatingFacesPerVertex.put(indices[i], vertexFacesSet);
         }
         else
         {
            vertexFacesSet = participatingFacesPerVertex.get(indices[i]);
         }

         vertexFacesSet.add(i / 3); // Abuse integer division.
      }

      Vector3f[] normalsPerFace = findNormalsPerFace(indices, vertices);

      int pos = 0;
      float[] normals = new float[3 * vertices.length];
      Vector3f vertexNormal, faceNormal;
      for (int vertexIndex = 0; vertexIndex < vertices.length; vertexIndex++)
      {
         Set<Integer> participatingFaceIndices = participatingFacesPerVertex.get(vertexIndex);
         vertexNormal = new Vector3f();
         for (Integer face : participatingFaceIndices)
         {
            faceNormal = normalsPerFace[face];
            vertexNormal.add(faceNormal);
         }
         vertexNormal.negate();
         float faces = (float) participatingFaceIndices.size();
         normals[pos++] = vertexNormal.x / faces;
         normals[pos++] = vertexNormal.y / faces;
         normals[pos++] = vertexNormal.z / faces;
      }

      return normals;
   }

   private static Vector3f[] findNormalsPerFace(int[] indices, Point3f[] vertices)
   {
      Vector3f[] normalsPerFace = new Vector3f[indices.length / 3]; // Abuse integer division.

      Vector3f firstVector = new Vector3f();
      Vector3f secondVector = new Vector3f();
      Point3f[] faceVertices = new Point3f[3];

      for (int face = 0; face < normalsPerFace.length; face++)
      {
         normalsPerFace[face] = new Vector3f();

         for (int i = 0; i < faceVertices.length; i++)
         {
            faceVertices[i] = vertices[indices[face * 3 + i]];
         }

         firstVector.set(faceVertices[2]);
         firstVector.sub(faceVertices[1]);

         secondVector.set(faceVertices[2]);
         secondVector.sub(faceVertices[0]);

         normalsPerFace[face].cross(firstVector, secondVector);
         normalsPerFace[face].normalize();
      }

      return normalsPerFace;
   }

   private static int[] splitPolygonIntoTriangles(int[] polygonIndices)
   {
      if (polygonIndices.length <= 3)
         return polygonIndices;

      // Do a naive way of splitting a polygon into triangles. Assumes convexity and ccw ordering.
      int[] ret = new int[3 * (polygonIndices.length - 2)];
      int i = 0;
      for (int j = 2; j < polygonIndices.length; j++)
      {
         ret[i++] = polygonIndices[0];
         ret[i++] = polygonIndices[j - 1];
         ret[i++] = polygonIndices[j];
      }

      return ret;
   }

   private static float[] convertToFloatArray(Tuple3f[] tuple3fs)
   {
      float[] array = new float[3 * tuple3fs.length];
      int index = 0;
      for (Tuple3f tuple : tuple3fs)
      {
         array[index++] = tuple.getX();
         array[index++] = tuple.getY();
         array[index++] = tuple.getZ();
      }
      return array;
   }

   private static float[] convertToFloatArray(Tuple2f[] tuple2fs)
   {
      float[] array = new float[2 * tuple2fs.length];
      int index = 0;
      for (Tuple2f tuple : tuple2fs)
      {
         array[index++] = tuple.getX();
         array[index++] = tuple.getY();
      }
      return array;
   }

   private static int[] doubleElements(int[] inputArray)
   {
      int[] outputArray = new int[inputArray.length * 3];
      int index = 0;
      for (int i : inputArray)
      {
         outputArray[index++] = i;
         outputArray[index++] = i;
         outputArray[index++] = i;
      }
      return outputArray;
   }
}
