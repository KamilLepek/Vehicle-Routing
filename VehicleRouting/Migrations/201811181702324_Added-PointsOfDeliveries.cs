namespace VehicleRouting.Migrations
{
    using System;
    using System.Data.Entity.Migrations;
    
    public partial class AddedPointsOfDeliveries : DbMigration
    {
        public override void Up()
        {
            CreateTable(
                "dbo.PointOfDeliveries",
                c => new
                    {
                        ID = c.Int(nullable: false, identity: true),
                        Name = c.String(),
                        CoordX = c.Single(nullable: false),
                        CoordY = c.Single(nullable: false),
                    })
                .PrimaryKey(t => t.ID);
            
        }
        
        public override void Down()
        {
            DropTable("dbo.PointOfDeliveries");
        }
    }
}
