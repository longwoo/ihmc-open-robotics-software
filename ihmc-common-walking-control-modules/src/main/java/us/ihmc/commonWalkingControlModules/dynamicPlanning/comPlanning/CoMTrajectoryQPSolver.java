package us.ihmc.commonWalkingControlModules.dynamicPlanning.comPlanning;

import org.ejml.data.DenseMatrix64F;
import org.ejml.ops.CommonOps;
import us.ihmc.commons.MathTools;
import us.ihmc.convexOptimization.quadraticProgram.AbstractSimpleActiveSetQPSolver;
import us.ihmc.convexOptimization.quadraticProgram.JavaQuadProgSolver;
import us.ihmc.convexOptimization.quadraticProgram.SimpleActiveSetQPSolverInterface;
import us.ihmc.robotics.linearAlgebra.MatrixTools;
import us.ihmc.robotics.time.ExecutionTimer;
import us.ihmc.yoVariables.providers.DoubleProvider;
import us.ihmc.yoVariables.registry.YoVariableRegistry;
import us.ihmc.yoVariables.variable.YoInteger;

public class CoMTrajectoryQPSolver
{
   private final YoVariableRegistry registry = new YoVariableRegistry(getClass().getSimpleName());

   private final CoMTrajectoryPlannerIndexHandler indexHandler;
   private final DoubleProvider omega;

   private final ExecutionTimer qpSolverTimer = new ExecutionTimer("qpSolverTimer", 0.5, registry);

   private final YoInteger numberOfIterations = new YoInteger("numberOfIterations", registry);
   private final YoInteger numberOfEqualityConstraints = new YoInteger("numberOfEqualityConstraints", registry);
   private final YoInteger numberOfInequalityConstraints = new YoInteger("numberOfInequalityConstraints", registry);
   private final YoInteger numberOfConstraints = new YoInteger("numberOfConstraints", registry);

   private final SimpleActiveSetQPSolverInterface qpSolver = new JavaQuadProgSolver();

   private final DenseMatrix64F solverInput_H;
   private final DenseMatrix64F solverInput_f;

   private final DenseMatrix64F solverInput_Aeq;
   private final DenseMatrix64F solverInput_beq;
   private final DenseMatrix64F solverInput_Ain;
   private final DenseMatrix64F solverInput_bin;

   private final DenseMatrix64F solverOutput;

   private final DenseMatrix64F tempJtW;

   public CoMTrajectoryQPSolver(CoMTrajectoryPlannerIndexHandler indexHandler, DoubleProvider omega, YoVariableRegistry parentRegistry)
   {
      this.omega = omega;
      this.indexHandler = indexHandler;

      int problemSize = indexHandler.getNumberOfVRPWaypoints();
      solverInput_H = new DenseMatrix64F(problemSize, problemSize);
      solverInput_f = new DenseMatrix64F(problemSize, 1);

      solverInput_Aeq = new DenseMatrix64F(0, problemSize);
      solverInput_beq = new DenseMatrix64F(0, 1);
      solverInput_Ain = new DenseMatrix64F(0, problemSize);
      solverInput_bin = new DenseMatrix64F(0, 1);

      solverOutput = new DenseMatrix64F(problemSize, 1);

      tempJtW = new DenseMatrix64F(problemSize, problemSize);

      parentRegistry.addChild(registry);
   }

   public void reset()
   {
      int problemSize = indexHandler.getNumberOfVRPWaypoints();

      solverInput_H.zero();

      solverInput_f.zero();

      solverInput_Aeq.reshape(0, problemSize);
      solverInput_beq.reshape(0, 1);

      solverInput_Ain.reshape(0, problemSize);
      solverInput_bin.reshape(0, 1);
   }

   public boolean solve()
   {
      numberOfEqualityConstraints.set(solverInput_Aeq.getNumRows());
      numberOfInequalityConstraints.set(solverInput_Ain.getNumRows());
      numberOfConstraints.set(solverInput_Aeq.getNumRows() + solverInput_Ain.getNumRows());

      qpSolverTimer.startMeasurement();

      qpSolver.clear();

      qpSolver.setQuadraticCostFunction(solverInput_H, solverInput_f, 0.0);
      qpSolver.setLinearInequalityConstraints(solverInput_Ain, solverInput_bin);
      qpSolver.setLinearEqualityConstraints(solverInput_Aeq, solverInput_beq);

      numberOfIterations.set(qpSolver.solve(solverOutput));

      qpSolverTimer.stopMeasurement();

      return !MatrixTools.containsNaN(solverOutput);
   }

   public void addTask(DenseMatrix64F taskJacobian, DenseMatrix64F taskObjective, DenseMatrix64F taskWeight)
   {
      addTaskInternal(taskJacobian, taskObjective, taskWeight, 0);
   }

   private void addTaskInternal(DenseMatrix64F taskJacobian, DenseMatrix64F taskObjective, DenseMatrix64F taskWeight, int offset)
   {
      int taskSize = taskJacobian.getNumRows();
      int variables = taskJacobian.getNumCols();
      if (offset + variables > indexHandler.getNumberOfVRPWaypoints())
      {
         throw new RuntimeException("This task does not fit.");
      }

      tempJtW.reshape(variables, taskSize);

      // J^T W
      CommonOps.multTransA(taskJacobian, taskWeight, tempJtW);

      // Compute: H += J^T W J
      MatrixTools.multAddBlock(tempJtW, taskJacobian, solverInput_H, offset, offset);

      // Compute: f += - J^T W Objective
      MatrixTools.multAddBlock(-1.0, tempJtW, taskObjective, solverInput_f, offset, 0);
   }

   static void populateWorkCostTermForSegment(int segmentNumber, double segmentDuration, double omega, double costWeight, DenseMatrix64F matrixToPack)
   {
      double omega3 = Math.pow(omega, 3);
      double omega4 = Math.pow(omega, 4);
      double omegaT = omega * segmentDuration;
      double eOmegaT = Math.exp(omegaT);
      double e2OmegaT = eOmegaT * eOmegaT;
      double eOmegaNegT = Math.exp(-omegaT);
      double e2OmegaNegT = eOmegaNegT * eOmegaNegT;

      int startIndex = 6 * segmentNumber;

      matrixToPack.set(startIndex + 0, startIndex + 0, costWeight * 0.5 * omega3 * (e2OmegaT - 1.0));
      matrixToPack.set(startIndex + 0, startIndex + 1, costWeight * omega4 * segmentDuration);
      matrixToPack.set(startIndex + 0, startIndex + 2, costWeight * 6.0 * (eOmegaT * (omegaT - 1) + 1));
      matrixToPack.set(startIndex + 0, startIndex + 3, costWeight * 2.0 * omega * (eOmegaT - 1.0));

      matrixToPack.set(startIndex + 1, startIndex + 0, matrixToPack.get(startIndex + 0, startIndex + 1));
      matrixToPack.set(startIndex + 1, startIndex + 1, costWeight * -0.5 * omega3 * (e2OmegaNegT - 1.0));
      matrixToPack.set(startIndex + 1, startIndex + 2, costWeight * -6.0 * (eOmegaNegT * (omegaT + 1) - 1));
      matrixToPack.set(startIndex + 1, startIndex + 3, costWeight * -2.0 * omega * (eOmegaNegT - 1.0));

      matrixToPack.set(startIndex + 2, startIndex + 0, matrixToPack.get(startIndex + 0, startIndex + 2));
      matrixToPack.set(startIndex + 2, startIndex + 1, matrixToPack.get(startIndex + 1, startIndex + 2));
      matrixToPack.set(startIndex + 2, startIndex + 2, costWeight * 12.0 * MathTools.pow(segmentDuration, 3));
      matrixToPack.set(startIndex + 2, startIndex + 3, costWeight * 6.0 * MathTools.square(segmentDuration));

      matrixToPack.set(startIndex + 3, startIndex + 0, matrixToPack.get(startIndex + 0, startIndex + 3));
      matrixToPack.set(startIndex + 3, startIndex + 1, matrixToPack.get(startIndex + 1, startIndex + 3));
      matrixToPack.set(startIndex + 3, startIndex + 2, matrixToPack.get(startIndex + 2, startIndex + 3));
      matrixToPack.set(startIndex + 3, startIndex + 3, costWeight * 4.0 * segmentDuration);
   }
}
