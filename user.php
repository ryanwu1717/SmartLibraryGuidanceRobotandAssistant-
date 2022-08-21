<?php
use Slim\Http\UploadedFile;
Class Move{
	var $result;
	var $conn;
	function __construct($db){
		$this->conn = $db;
	}
	function getMove(){
		$sql ='SELECT * FROM move ORDER BY  `time` DESC limit 1 ;';
		$sth = $this->conn->prepare($sql);

		$sth->execute();
		$row = $sth->fetchAll();
		return $row;
	}

	function getCount(){
		$sql ='SELECT count(*) as `count` FROM move WHERE ;';
		$sth = $this->conn->prepare($sql);
		$sth->execute();
		$row = $sth->fetchAll();
		return $row;
	}

	function getFinish(){
		$sql ='SELECT count(*) as `count` FROM move WHERE `finish` = 1;';
		$sth = $this->conn->prepare($sql);
		$sth->execute();
		$row = $sth->fetchAll();
		return $row;
	}

	function finish($id){
		// $body=json_decode($_POST['data'],true);
		$sql ='UPDATE move SET finish = :finish
				WHERE id=:id;';
		$sth = $this->conn->prepare($sql);
		$finish = 1;
   	$sth->bindParam(':id',$id);
   	$sth->bindParam(':finish',$finish);
		$sth->execute();
		$ack = array(
			'status' => $id
		);
		return $ack;
	}

	function insert($body){
		$body=json_decode($body['data'],true);
		$finish = 0;
		$sql ='INSERT INTO move (`time`, `oldX`, `oldY`,`newX`,`newY`,`finish`)
				VALUES (NOW(),:oldX, :oldY,:newX,:newY,:finish);';
		$sth = $this->conn->prepare($sql);
   	$sth->bindParam(':oldX',$body['tmpX']);
   	$sth->bindParam(':oldY',$body['tmpY']);
   	$sth->bindParam(':newX',$body['newX']);
   	$sth->bindParam(':newY',$body['newY']);
   	$sth->bindParam(':finish',$finish);
		$sth->execute();
		// $row = $sth->fetchAll();
		// var_dump($_SESSION['id']);
		
		$ack = array(
			'status' => 'success'
		);
		return $ack;
		// return $body;

	}
	
}
?>
