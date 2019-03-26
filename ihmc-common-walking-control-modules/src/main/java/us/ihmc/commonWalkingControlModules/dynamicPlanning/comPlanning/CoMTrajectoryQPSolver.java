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
   private final CoMTrajectoryPlannerIndexHandler indexHandler;
   private final DoubleProvider omega;

   private final ExecutionTimer qpSolverTimer;

   private final YoInteger numberOfIterations;
   private final YoInteger numberOfEqualityConstraints;
   private final YoInteger numberOfInequalityConstraints;
   private final YoInteger numberOfConstraints;

   private final SimpleActiveSetQPSolverInterface qpSolver = new JavaQuadProgSolver();

   private final DenseMatrix64F solverInput_H;
   private final DenseMatrix64F solverInput_f;

   private final DenseMatrix64F solverInput_Aeq;
   private final DenseMatrix64F solverInput_beq;
   private final DenseMatrix64F solverInput_Ain;
   private final DenseMatrix64F solverInput_bin;

   private final DenseMatrix64F solverOutput;

   private final DenseMatrix64F tempJtW;

   public CoMTrajectoryQPSolver(String prefix, CoMTrajectoryPlannerIndexHandler indexHandler, DoubleProvider omega, YoVariableRegistry parentRegistry)
   {
      this.omega = omega;
      this.indexHandler = indexHandler;

      YoVariableRegistry registry = new YoVariableRegistry(prefix + getClass().getSimpleName());
      qpSolverTimer = new ExecutionTimer(prefix + "QpSolverTimer", 0.5, registry);
      numberOfIterations = new YoInteger(prefix + "NumberOfIterations", registry);
      numberOfEqualityConstraints = new YoInteger(prefix + "NumberOfEqualityConstraints", registry);
      numberOfInequalityConstraints = new YoInteger(prefix + "NumberOfInequalityConstraints", registry);
      numberOfConstraints = new YoInteger(prefix + "NumberOfConstraints", registry);

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

      solverInput_H.reshape(problemSize, problemSize);
      solverInput_f.reshape(problemSize, 1);

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

   public void addTask(DenseMatrix64F taskJacobian, DenseMatrix64F taskObjective, double taskWeight)
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

   private void addTaskInternal(DenseMatrix64F taskJacobian, DenseMatrix64F taskObjective, double taskWeight, int offset)
   {
      int variables = taskJacobian.getNumCols();
      if (offset + variables > indexHandler.getNumberOfVRPWaypoints())
      {
         throw new RuntimeException("This task does not fit.");
      }

      // Compute: H += J^T W J
      MatrixTools.multAddBlockInner(taskWeight, taskJacobian, solverInput_H, offset, offset);

      // Compute: f += - J^T W Objective
      MatrixTools.multAddBlockTransA(-taskWeight, taskJacobian, taskObjective, solverInput_f, offset, 0);
   }

   public void getSolution(DenseMatrix64F solutionToPack)
   {
      solutionToPack.set(solverOutput);
   }

}
