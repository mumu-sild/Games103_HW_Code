最新内容请到https://blog.csdn.net/weixin_44518102/article/details/123247073 查看

# 作业框架已知条件

1. 兔子模型的顶点集
2. 每个墙体的上的点P及法线N；
3. 兔子初始速度（角速度w&线速度v）
4. 全局速度衰减系数(linear_decay，angular_decay)，全局弹性系数（restitution），全局摩擦系数（friction）
5. 步时（dt）
6. 重力加速度（gravity）
7. 惯性张量（I_ref）

```cpp
	bool launched 		= false;
	float dt 			= 0.015f;
	Vector3 v 			= new Vector3(0, 0, 0);	// velocity
	Vector3 w 			= new Vector3(0, 0, 0);	// angular velocity
	
	float mass;									// mass
	Matrix4x4 I_ref;							// reference inertia

	float linear_decay	= 0.999f;				// for velocity decay速度衰减
	float angular_decay	= 0.98f;				
	float restitution 	= 0.5f;                 // for collision 弹性系数
	float friction		= 0.2f;					// 摩擦系数

	Vector3 gravity = new Vector3(0.0f, -9.8f, 0.0f);
```
# 逻辑梳理
## 1.当用户按下r键后，物体获得初始速度
当物体获得初始速度后，首先需要计算下一时刻的速度，而下一时刻的速度变化与该物体受到的力有关。

其中包括：重力，空气摩擦力，弹力，摩擦力等

通过这些力可以计算速度的变化量，从而得到下一时刻的速度。

## 2.修改当前速度为下一时刻速度
### （1）重力，空气摩擦力
其中重力为非彻体力，空气摩擦力可近似为一定系数的速度衰减，故有：

```cpp
// Part I: Update velocities
			v += dt * gravity;
			v *= linear_decay;
			w *= angular_decay;
```
### （2）由碰撞导致的速度变化
碰撞导致的速度变化首先得确定**是否发生碰撞**。

-----------------------------------

#### ①碰撞检测
对兔子模型的每一个点$x_i$,有：
$$d = (x_i - p)N$$
若 $d<0$,则说明点在平面内，即发生了碰撞。

**注：这种碰撞检测方法仅适用于无线长的平面检测。**

--------------------------------

#### ②求碰撞发生的平均碰撞点
因为一个模型与另一个模型发生碰撞往往不会只有一个点发生碰撞，通常这与**模型的精度**，**两模型的相对速度**，和**步时**有关。

若将步时修改为0.01（标准值的$\frac{1}{15}$），发生碰撞时的的碰撞点有18个左右。而此时的时间精度相对于游戏或影视来说已经绰绰有余。单纯的通过提高时间精度来获得更少的接触点来计算，显然显得费力费时。

而我们知道，一个物体与另一个物体的碰撞可以在宏观上看作点的碰撞，微观上看作面与面的碰撞。这其实与我们的条件略有相似。
若我们将如上的18个碰撞点当作碰撞面（发生微小形变），因此在宏观上便有一个近似的虚拟碰撞点来作为面与面碰撞的近似。

**这便引出了平均碰撞点（或虚拟碰撞点）的使用。**

由于这些碰撞点中有的碰撞点速度在平面法线方向的速度为正，即对平面并没有冲量，因此在计算中需要删除这些点。~~具体我物理道理我也没想明白。~~ 

故得出平均碰撞点的代码为：

```cpp
for (int i = 0; i < vertices.Length; i++)
        {
			//3.计算每个顶点到该表面的距离d
			Vector3 r_i = vertices[i];
			Vector3 Rri = R.MultiplyVector(r_i);
			Vector3 x_i =  T + Rri;
			float d = Vector3.Dot(x_i - P , N);
            if (d < 0.0f)
            {
				//4.判断物体是否仍向墙体内部运动
				Vector3 v_i = v + Vector3.Cross(w, Rri);
				float v_N_size = Vector3.Dot(v_i, N);
				if (v_N_size < 0.0f)
                {
					sum += r_i;
					collisionNum++;
				}
            }
        }
		if (collisionNum == 0) return;
		Vector3 r_collision = sum / collisionNum;                //平均碰撞点（局部坐标）
		Vector3 Rr_collision = R.MultiplyVector(r_collision);
		Vector3 v_collision = v + Vector3.Cross(w, Rr_collision);//平均碰撞点的速度分量（世界坐标）
```

------------------------------------------

#### ③计算碰撞发生后的新速度
首先将世界坐标系下**平均碰撞点**的速度分解为**法向速度**和**切向速度**。

分解公式如下：

$$\mathbf {v_N} = (\mathbf v \cdot \mathbf N)\mathbf N$$
$$\mathbf {v_T} = \mathbf v - \mathbf {v_N}$$

我们需要通过 $\mu_N$ , 弹性系数 $\mu_T$ 摩擦系数改变碰撞平均点的速度。根据如下关系
$\mathbf {v_N^{new}} = - \mu_{\mathbf N} \mathbf {v_N}$
$\mathbf {v_T^{new}} = a \mathbf {v_T}$  
根据物理原理，可以推导出
$a = max(1-\mu_T(1+\mu_N)||\mathbf {v_N}||/|| \mathbf{v_T}|| , 0)$
所以，最终平均碰撞点新的速度为
$\mathbf {v^{new}} = \mathbf v_N^{new} + \mathbf {v_T^{new}}$ 

代码为：
- 其中 **restitution** 为$\mu_N$， **friction** 为$\mu_T$.

```csharp
Vector3 v_N = Vector3.Dot(v_collision, N) * N;
Vector3 v_T = v_collision - v_N;
Vector3 v_N_new = -1.0f * restitution * v_N;
float a = Math.Max(1.0f - friction * (1.0f + restitution) * v_N.magnitude / 
					v_T.magnitude, 0.0f);
Vector3 v_T_new = a * v_T;
Vector3 v_new = v_N_new + v_T_new;
```
#### ④通过新旧速度计算碰撞对模型的冲量
##### 问题：为什么不能直接用新速度更新模型
因为新速度是模型上的一个点（或多个点）以质点（平均碰撞点）的形式与墙体碰撞，并以一种计算方法得到该质点碰撞后的新速度。而这个新速度在整个模型上需要转化为该点角速度和线速度的和。因此我们需要引入冲量 $J$ 的概念，来计算模型的线速度与角速度。

##### 冲量J的计算方法证明
因为模型任意一点$\mathbf x_i$有
  $\mathbf v^{new}_i = \mathbf v^{new} +\omega^{new}\times R\mathbf {r_i}$
  而
 $\mathbf v^{new} = \mathbf v + \frac{1}{M}\mathbf j$
 $\omega^{new} = \omega + \mathbf I^{-1}(R\mathbf{r_i}\times \mathbf j)$
![在这里插入图片描述](Readme.assets/watermark,type_d3F5LXplbmhlaQ,shadow_50,text_Q1NETiBARWxzYeeahOi_t-W8nw==,size_20,color_FFFFFF,t_70,g_se,x_16.png)
去除叉乘运算，转化为矩阵点乘，有：
![在这里插入图片描述](Readme.assets/63fc2b988dd24d04a21201fb8985904c.png)
最后得到：
$\mathbf {v_i^{new}} = \mathbf{v_i+Kj}$
其中
$\mathbf K = \frac{1}{M}\mathbf 1 - (R\mathbf {r_i})^*I^{-1}(R \mathbf {r_i})^*$

因为$\mathbf {v_i^{new}}，\mathbf{v_i}，\mathbf K$已知，因此可通过这些来推出$\mathbf j$。
即
$$\mathbf j =\mathbf K^{-1}(\mathbf {v_i^{new}} - \mathbf{v_i})$$
$$\mathbf K = \frac{1}{M}\mathbf 1 - (R\mathbf {r_i})^*I^{-1}(R \mathbf {r_i})^*$$

##### 代码
```csharp
Matrix4x4 Rri_star = Get_Cross_Matrix(Rr_collision);
Matrix4x4 K = Matrix_subtraction(Matrix_miltiply_float(Matrix4x4.identity, 1.0f / mass),
										Rri_star * I_inverse * Rri_star);
Vector3 J = K.inverse.MultiplyVector(v_new - v_collision);
```

#### ⑤通过冲量修改模型速度
 $\mathbf v^{new} = \mathbf v + \frac{1}{M}\mathbf j$
 $\omega^{new} = \omega + \mathbf I^{-1}(R\mathbf{r_i}\times \mathbf j)$
```csharp
v += 1.0f / mass * J;
w += I_inverse.MultiplyVector(Vector3.Cross(Rr_collision, J));
```
## 3.更新模型位置和朝向
使用 Leapfrog Integration(蛙跳法积分方法)更新位移关系。
$\mathbf v_{0.5} = \mathbf v_{-0.5} + \bigtriangleup t M^{-1}\mathbf{f}_{0}$
$\mathbf x_1 =\mathbf x_0 + \bigtriangleup t\mathbf v_{0.5}$
假设 $\mathbf v_初 = \mathbf v_{-0.5}$，个人认为这里和欧拉积分并无差别。

使用旋转的更新法则更新 $\omega$.
$\omega_1 = \omega_0 + Δt(I_0)^{-1}\tau_0$
$\mathbf q_1 = \mathbf q_0 + [0, \frac{Δt}{2}\omega_1]\times\mathbf q_0$

即有：

```csharp
// Part III: Update position & orientation
Vector3 x_0 = transform.position;
Quaternion q_0 = transform.rotation;
//Update linear status
Vector3 x = x_0 + dt * v;
//Update angular status
Vector3 dw = 0.5f * dt * w;
Quaternion qw = new Quaternion(dw.x, dw.y, dw.z, 0.0f);
Quaternion q = Add(q_0, qw * q_0);
```
# 整体代码

```csharp
using UnityEngine;
using System.Collections;
using System;

public class Rigid_Bunny : MonoBehaviour 
{
	bool launched 		= false;
	float dt 			= 0.015f;
	Vector3 v 			= new Vector3(0, 0, 0);	// velocity
	Vector3 w 			= new Vector3(0, 0, 0);	// angular velocity
	
	float mass;									// mass
	Matrix4x4 I_ref;							// reference inertia

	float linear_decay	= 0.999f;				// for velocity decay速度衰减
	float angular_decay	= 0.98f;				
	float restitution 	= 0.5f;                 // for collision 弹性系数
	float friction		= 0.2f;					// 摩擦系数

	Vector3 gravity = new Vector3(0.0f, -9.8f, 0.0f);


	// Use this for initialization
	void Start () 
	{		
		Mesh mesh = GetComponent<MeshFilter>().mesh;
		Vector3[] vertices = mesh.vertices;

		float m=1;
		mass=0;
		for (int i=0; i<vertices.Length; i++) 
		{
			mass += m;
			float diag=m*vertices[i].sqrMagnitude;//diag = mv^2
			I_ref[0, 0]+=diag;
			I_ref[1, 1]+=diag;
			I_ref[2, 2]+=diag;
			I_ref[0, 0]-=m*vertices[i][0]*vertices[i][0];
			I_ref[0, 1]-=m*vertices[i][0]*vertices[i][1];
			I_ref[0, 2]-=m*vertices[i][0]*vertices[i][2];
			I_ref[1, 0]-=m*vertices[i][1]*vertices[i][0];
			I_ref[1, 1]-=m*vertices[i][1]*vertices[i][1];
			I_ref[1, 2]-=m*vertices[i][1]*vertices[i][2];
			I_ref[2, 0]-=m*vertices[i][2]*vertices[i][0];
			I_ref[2, 1]-=m*vertices[i][2]*vertices[i][1];
			I_ref[2, 2]-=m*vertices[i][2]*vertices[i][2];
		}
		I_ref [3, 3] = 1;
	}
	
	Matrix4x4 Get_Cross_Matrix(Vector3 a)//得到向量a的叉乘矩阵
	{
		//Get the cross product matrix of vector a
		Matrix4x4 A = Matrix4x4.zero;
		A [0, 0] = 0; 
		A [0, 1] = -a [2]; 
		A [0, 2] = a [1]; 
		A [1, 0] = a [2]; 
		A [1, 1] = 0; 
		A [1, 2] = -a [0]; 
		A [2, 0] = -a [1]; 
		A [2, 1] = a [0]; 
		A [2, 2] = 0; 
		A [3, 3] = 1;
		return A;
	}

	private Quaternion Add(Quaternion a, Quaternion b)
	{
		a.x += b.x;
		a.y += b.y;
		a.z += b.z;
		a.w += b.w;
		return a;
	}

	private Matrix4x4 Matrix_subtraction(Matrix4x4 a, Matrix4x4 b)
    {
		for (int i = 0; i < 4; ++i)
		{
			for (int j = 0; j < 4; ++j)
			{
				a[i, j] -= b[i,j];
			}
		}
		return a;
	}


	private Matrix4x4 Matrix_miltiply_float(Matrix4x4 a,float b)
    {
		for(int i = 0; i < 4; ++i)
        {
			for(int j = 0; j < 4; ++j)
            {
				a[i, j] *= b; 
            }
        }
		return a;
    }

	// In this function, update v and w by the impulse due to the collision with
	//a plane <P, N>
	//P 为该平面上的一个点，N为该平面的法线
	void Collision_Impulse(Vector3 P, Vector3 N)
	{
		//1.获取物体的每一个顶点(局部坐标)
		Mesh mesh = GetComponent<MeshFilter>().mesh;
		Vector3[] vertices = mesh.vertices;

		//2.得到每一个顶点的全局坐标旋转矩阵R,和平移向量
		Matrix4x4 R = Matrix4x4.Rotate(transform.rotation);  //旋转矩阵
		Vector3 T = transform.position;						 //平移向量

		Vector3 sum = new Vector3(0,0,0);					//碰撞点
		int collisionNum = 0;								//碰撞点数量


		for (int i = 0; i < vertices.Length; i++)
        {
			//3.计算每个顶点到该表面的距离d
			Vector3 r_i = vertices[i];
			Vector3 Rri = R.MultiplyVector(r_i);
			Vector3 x_i =  T + Rri;
			float d = Vector3.Dot(x_i - P , N);
            if (d < 0.0f)//发生碰撞(只有当平面为无限大平面时才能这样判断，否则还要判断碰撞点是否在物体上)
            {
				//4.将该点移动到距离表面最近的点。?????
				//x_i -= d * N
				//5.判断物体是否仍向墙体内部运动
				Vector3 v_i = v + Vector3.Cross(w, Rri);
				float v_N_size = Vector3.Dot(v_i, N);
				if (v_N_size < 0.0f)
                {
					sum += r_i;
					collisionNum++;
				}
            }
        }
		//Debug.LogFormat("共有{0}个点", collisionNum);

		if (collisionNum == 0) return;
		Matrix4x4 I_rot = R * I_ref * Matrix4x4.Transpose(R);//惯性张量（全局）
		Matrix4x4 I_inverse = Matrix4x4.Inverse(I_rot);      //惯性张量的逆（全局）
		Vector3 r_collision = sum / collisionNum;                //虚拟碰撞点（局部坐标）
		Vector3 Rr_collision = R.MultiplyVector(r_collision);
		//Vector3 x_collision = T + Rr_collision;							 //虚拟碰撞点（全局坐标）
		Vector3 v_collision = v + Vector3.Cross(w, Rr_collision);
		//6.如果物体仍向墙体内部运动,修改为新速度
		Vector3 v_N = Vector3.Dot(v_collision, N) * N;
		Vector3 v_T = v_collision - v_N;
		Vector3 v_N_new = -1.0f * restitution * v_N;
		float a = Math.Max(1.0f - friction * (1.0f + restitution) * v_N.magnitude / v_T.magnitude, 0.0f);
		Vector3 v_T_new = a * v_T;
		Vector3 v_new = v_N_new + v_T_new;
		//7.通过新速度量计算冲量J
		Matrix4x4 Rri_star = Get_Cross_Matrix(Rr_collision);
		Matrix4x4 K = Matrix_subtraction(Matrix_miltiply_float(Matrix4x4.identity, 1.0f / mass),
										Rri_star * I_inverse * Rri_star);
		Vector3 J = K.inverse.MultiplyVector(v_new - v_collision);
		//8.计算dv,dw;
		v += 1.0f / mass * J;
		w += I_inverse.MultiplyVector(Vector3.Cross(Rr_collision, J));
		//9.通过冲量J改变整个物体的线速度和角速度
	}

    // Update is called once per frame
    void Update () 
	{
		//Game Control
		if(Input.GetKey("r"))
		{
			transform.position = new Vector3 (0, 0.6f, 0);
			restitution = 0.5f;
			launched=false;
			Debug.Log("返回原点");
		}
		if(Input.GetKey("l"))
		{
			v = new Vector3 (5, 2, 0);
			w = new Vector3 (0, 1, 0);   
			launched =true;
		}

		if (launched)
		{
			// Part I: Update velocities
			v += dt * gravity;
			v *= linear_decay;
			w *= angular_decay;

			// Part II: Collision Impulse
			Collision_Impulse(new Vector3(0, 0.01f, 0), new Vector3(0, 1, 0));
			Collision_Impulse(new Vector3(2, 0, 0), new Vector3(-1, 0, 0));

			// Part III: Update position & orientation
			Vector3 x_0 = transform.position;
			Quaternion q_0 = transform.rotation;
			//Update linear status
			Vector3 x = x_0 + dt * v;
			//Update angular status
			Vector3 dw = 0.5f * dt * w;
			Quaternion qw = new Quaternion(dw.x, dw.y, dw.z, 0.0f);
			Quaternion q = Add(q_0, qw * q_0);

			// Part IV: Assign to the object
			transform.position = x;
			transform.rotation = q;
		}
	}
}

```
