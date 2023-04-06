package example

import (
	"fmt"
	"log"
	"testing"

	"github.com/ByteArena/box2d"
)

// https://juejin.cn/post/6844903448371003406
func GetWorld() *box2d.B2World {
	// Define the gravity vector.
	gravity := box2d.MakeB2Vec2(0.0, -10.0)
	// Construct a world object, which will hold and simulate the rigid bodies.
	world := box2d.MakeB2World(gravity)
	return &world
}

func SetBody1(world *box2d.B2World) *box2d.B2Body {
	bodyDef := box2d.NewB2BodyDef() // 刚体实例化
	bodyDef.Type = box2d.B2BodyType.B2_dynamicBody
	bodyDef.Awake = true
	fixDef := box2d.NewB2Fixture()
	fixDef.SetDensity(1.0)
	fixDef.SetFriction(0.9)
	fixDef.SetRestitution(1.0)
	fixDef.M_shape = box2d.NewB2CircleShape()
	fixDef.M_shape.(*box2d.B2CircleShape).SetRadius(50 / PTM_RATIO)
	bodyDef.Position.Set(250/PTM_RATIO, 0/PTM_RATIO)
	body := world.CreateBody(bodyDef)
	body.CreateFixture(fixDef.M_shape, fixDef.M_density) // 为 body 套上材质

	return body
}

func SetBody2InWorld(world *box2d.B2World) *box2d.B2Body {
	bodyDef := box2d.NewB2BodyDef() // 刚体实例化
	bodyDef.Type = box2d.B2BodyType.B2_dynamicBody
	//bodyDef.Type = box2d.B2BodyType.B2_kinematicBody
	bodyDef.Awake = true
	bodyDef.Position.Set(0., 4.)
	body := world.CreateBody(bodyDef)
	mass := box2d.NewMassData()
	mass.Mass = 10
	body.SetMassData(mass)

	dynBox := box2d.NewB2PolygonShape()
	dynBox.SetAsBox(10., 10.)

	fixDef := box2d.NewB2Fixture()
	// 密度
	fixDef.SetDensity(1.0)
	// 摩擦系数
	fixDef.SetFriction(0.3)
	// 恢复系数
	fixDef.SetRestitution(1.0)
	// 是否为传感器：当设置为 true 时，刚体发生碰撞的时候并不会发生碰撞响应（反弹），但是会接收到碰撞的信号，所以该属性可以理解为传感器。
	fixDef.SetSensor(false)
	fixDef.M_shape = dynBox
	// 为 body 套上材质
	body.CreateFixture(fixDef.M_shape, fixDef.M_density)
	body.SetLinearVelocity(box2d.B2Vec2{
		X: 0,
		Y: 1,
	})
	body.SetUserData(map[string]any{
		"id": 2,
	})

	return body
}

func SetBody3InWorld(world *box2d.B2World) *box2d.B2Body {
	bodyDef := box2d.NewB2BodyDef() // 刚体实例化
	bodyDef.Type = box2d.B2BodyType.B2_kinematicBody
	// 设置位置 cpp_compliance_test.go:166
	bodyDef.Position.Set(0., 0.)
	body := world.CreateBody(bodyDef)
	mass := box2d.NewMassData()
	mass.Mass = 10
	body.SetMassData(mass)

	dynBox := box2d.NewB2PolygonShape()
	dynBox.SetAsBox(15., 15.)

	fixDef := box2d.NewB2Fixture()
	// 密度
	fixDef.SetDensity(1.0)
	// 摩擦系数
	fixDef.SetFriction(0.3)
	// 恢复系数
	fixDef.SetRestitution(1.0)
	// 是否为传感器：当设置为 true 时，刚体发生碰撞的时候并不会发生碰撞响应（反弹），但是会接收到碰撞的信号，所以该属性可以理解为传感器。
	fixDef.SetSensor(false)
	fixDef.M_shape = dynBox

	body.CreateFixture(fixDef.M_shape, fixDef.M_density) // 为 body 套上材质

	body.SetLinearVelocity(box2d.B2Vec2{
		X: 0,
		Y: 100,
	})
	body.SetUserData(map[string]any{
		"id": 3,
	})

	return body
}

type LocalContactListener struct{}

func (_this *LocalContactListener) BeginContact(contact box2d.B2ContactInterface) {
	log.Printf("[BeginContact] ...")
}
func (_this *LocalContactListener) EndContact(contact box2d.B2ContactInterface) {

}
func (_this *LocalContactListener) PreSolve(contact box2d.B2ContactInterface, oldManifold box2d.B2Manifold) {

}
func (_this *LocalContactListener) PostSolve(contact box2d.B2ContactInterface, impulse *box2d.B2ContactImpulse) {

}

// world 的 hello world 示例。物体的运动
func TestHelloWorld1(t *testing.T) {
	var timeStep float64 = 1. / 60.
	timeStep = 20.
	world := GetWorld()
	SetBody2InWorld(world)
	SetBody3InWorld(world)
	// last := time.Now().UnixMilli()
	var animate func()
	animate = func() {
		// nowTime := time.Now()
		// var timeStep = nowTime.UnixMilli() - last
		// last = nowTime.UnixMilli()
		// velocityIterations 是对速率的纠正程度, 越高计算量越大.
		var velocityIterations = 6
		// positionIterations 是对位置的纠正程度, 越高计算量越大.
		var positionIterations = 2
		world.Step(timeStep, velocityIterations, positionIterations)
		//world.ClearForces()
	}
	world.SetContactListener(&LocalContactListener{})
	for i := 0; i < 1000; i++ {
		animate()

		bodys := world.GetBodyList()
		for fObj := bodys.GetFixtureList().GetBody(); fObj != nil; fObj = fObj.GetNext() {
			userData := fObj.GetUserData().(map[string]any)
			bodyId := userData["id"]
			fmt.Printf("body id: %v,  x: %v, y: %v \n", bodyId, fObj.GetPosition().X, fObj.GetPosition().Y)

			cl := fObj.GetContactList()
			if cl != nil {
				fmt.Printf("GetContactList: %v", cl)
			}
		}

		for list := bodys.GetContactList(); list != nil; list = list.Next {
			fmt.Printf("body v: %v \n", list.Contact)
		}

		for cnArr := world.GetContactList(); cnArr != nil; cnArr.GetNext() {
			bodyA := cnArr.GetFixtureA()
			bodyB := cnArr.GetFixtureB()

			ta := bodyA.GetBody().GetTransform()
			tb := bodyB.GetBody().GetTransform()
			touching := box2d.B2TestOverlapShapes(bodyA.M_shape, cnArr.GetChildIndexA(), bodyB.M_shape, cnArr.GetChildIndexB(), ta, tb)
			if touching {
				posi := bodyA.GetBody().GetPosition()
				angle := bodyA.GetBody().GetAngle()
				fmt.Printf("res posi x: %v, y:%v, angle:%v \n", posi.X, posi.Y, angle)
			}
			continue

			posi := bodyA.GetBody().GetPosition()
			angle := bodyA.GetBody().GetAngle()
			fmt.Printf("res posi x: %v, y:%v, angle:%v \n", posi.X, posi.Y, angle)
			if bodyA.IsSensor() {
				world.DestroyBody(bodyA.M_body)
				log.Printf("destroy a")
			}

			posi = bodyB.GetBody().GetPosition()
			angle = bodyB.GetBody().GetAngle()
			fmt.Printf("res posi x: %v, y:%v, angle:%v \n", posi.X, posi.Y, angle)
			if bodyB.IsSensor() {
				world.DestroyBody(bodyB.M_body)
				log.Printf("destroy b")
			}
		}
	}
}

// ref https://www.jianshu.com/p/7681431618ec
func NewDefShape1() {
	// 三角形
	// 描述三角形的三个点的坐标
	vertiArr := make([]box2d.B2Vec2, 3)
	vertiArr[0].Set(0., 0.)
	vertiArr[1].Set(1., 0.)
	vertiArr[2].Set(0., 1.)
	cnt := 3
	polygon := box2d.MakeB2PolygonShape()
	polygon.Set(vertiArr, cnt)

	// edge shape
	// 为了帮助为游戏创建自由形式的静态环境。边缘形状没有体积
	edge1 := box2d.MakeB2EdgeShape()
	edge1.Set(box2d.B2Vec2{0., 0.}, box2d.B2Vec2{0., 0.})
}
