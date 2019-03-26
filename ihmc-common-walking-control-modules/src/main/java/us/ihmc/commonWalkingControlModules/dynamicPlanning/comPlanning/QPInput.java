package us.ihmc.commonWalkingControlModules.dynamicPlanning.comPlanning;

import org.ejml.data.DenseMatrix64F;
import us.ihmc.commonWalkingControlModules.controllerCore.command.ConstraintType;

public class QPInput
{
   private static final int initialTaskSize = 6;

   private final int numberOfVariables;

   public final DenseMatrix64F taskJacobian = new DenseMatrix64F(0, 0);
   public final DenseMatrix64F taskXObjective = new DenseMatrix64F(0, 0);
   public final DenseMatrix64F taskYObjective = new DenseMatrix64F(0, 0);
   public final DenseMatrix64F taskZObjective = new DenseMatrix64F(0, 0);
   public final DenseMatrix64F taskWeightMatrix = new DenseMatrix64F(0, 0);

   private boolean useWeightScalar = false;
   private double taskWeightScalar;

   private ConstraintType constraintType = ConstraintType.OBJECTIVE;

   /**
    * <p>
    * Input into the QP solver. Must be in the form
    * </p>
    * <p>
    * A * x - b
    * </p>
    * where:
    * <ul>
    * <li>A is {@code taskJacobian}
    * <li>b is {@code taskObjective}
    * <li>x is the vector of the problem variables, for instance joint accelerations.
    * <p>
    * where the overall desire is minimize the objective.
    * </p>
    */
   public QPInput(int numberOfVariables)
   {
      this.numberOfVariables = numberOfVariables;
      reshape(initialTaskSize, numberOfVariables);
   }

   public void reshape(int taskSize, int numberOfVariables)
   {
      taskJacobian.reshape(taskSize, numberOfVariables);
      taskXObjective.reshape(taskSize, 1);
      taskYObjective.reshape(taskSize, 1);
      taskZObjective.reshape(taskSize, 1);
      taskWeightMatrix.reshape(taskSize, taskSize);

      taskJacobian.zero();
      taskXObjective.zero();
      taskYObjective.zero();
      taskZObjective.zero();
      taskWeightMatrix.zero();
   }

   public void setTaskJacobian(DenseMatrix64F taskJacobian)
   {
      this.taskJacobian.set(taskJacobian);
   }

   public DenseMatrix64F getTaskJacobian()
   {
      return taskJacobian;
   }

   public void setTaskXObjective(DenseMatrix64F taskObjective)
   {
      this.taskXObjective.set(taskObjective);
   }

   public void setTaskYObjective(DenseMatrix64F taskObjective)
   {
      this.taskYObjective.set(taskObjective);
   }

   public void setTaskZObjective(DenseMatrix64F taskObjective)
   {
      this.taskZObjective.set(taskObjective);
   }

   public DenseMatrix64F getTaskXObjective()
   {
      return taskXObjective;
   }

   public DenseMatrix64F getTaskYObjective()

   {
      return taskYObjective;
   }

   public DenseMatrix64F getTaskZObjective()
   {
      return taskZObjective;
   }

   public void setTaskWeightMatrix(DenseMatrix64F taskWeightMatrix)
   {
      this.taskWeightMatrix.set(taskWeightMatrix);
   }

   public DenseMatrix64F getTaskWeightMatrix()
   {
      return taskWeightMatrix;
   }

   public void setUseWeightScalar(boolean useWeightScalar)
   {
      this.useWeightScalar = useWeightScalar;
   }

   public void setWeight(double weight)
   {
      this.taskWeightScalar = weight;
   }

   public double getWeightScalar()
   {
      return taskWeightScalar;
   }

   public double getTaskWeightScalar()
   {
      return taskWeightScalar;
   }

   public boolean useWeightScalar()
   {
      return useWeightScalar;
   }

   public void setConstraintType(ConstraintType constraintType)
   {
      this.constraintType = constraintType;
   }

   public ConstraintType getConstraintType()
   {
      return constraintType;
   }

   @Override
   public String toString()
   {
      String ret = getClass().getSimpleName();
      ret += "Jacobian:\n" + taskJacobian;
      ret += "X Objective:\n" + taskXObjective;
      ret += "Y Objective:\n" + taskYObjective;
      ret += "Z Objective:\n" + taskZObjective;
      if (constraintType != ConstraintType.OBJECTIVE)
         ret += constraintType.toString();
      else if (useWeightScalar)
         ret += "Weight: " + taskWeightScalar;
      else
         ret += "Weight:\n" + taskWeightMatrix;
      return ret;
   }
}
