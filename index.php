<?php
use \Psr\Http\Message\ServerRequestInterface as Request;
use \Psr\Http\Message\ResponseInterface as Response;

require '../vendor/autoload.php';
require_once __DIR__.'/../model/user.php';

use Slim\Views\PhpRenderer;

$configuration = [
    'settings' => [
        'displayErrorDetails' => true
    ],
];

$c = new \Slim\Container($configuration);
$app = new \Slim\App($c);

$container = $app->getContainer();


$container['view'] = function ($container) {
    return new PhpRenderer(__DIR__.'/../view');
};
$container['notFoundHandler'] = function ($container) {
    return function ($request, $response) use ($container) {
        return $response->withStatus(404)
            ->withHeader('Content-Type', 'text/html')
            ->write('Page not found');
    };
};
$container['db'] = function ($container) {
	$dbhost = '127.0.0.1';
	// $dbport = '3306';
	$dbuser = 'root';
	$dbpasswd = '7172930';
	$dbname = 'robot';
	$dsn = "mysql:host=".$dbhost.";dbname=".$dbname;
	try
	{

	    $conn = new \PDO($dsn,$dbuser,$dbpasswd);
	    $conn->exec("SET CHARACTER SET utf8");
	    $conn->setAttribute(PDO::ATTR_ERRMODE, PDO::ERRMODE_EXCEPTION);
	    // echo "Connected Successfully";
	    // echo "success";
	}
	catch(PDOException $e)
	{
	    echo "Connection failed: ".$e->getMessage();
	}
	return $conn;
};

// $app = new \Slim\App(['settings' => ['displayErrorDetails' => true]]);
// $app = new \Slim\App;
$app->group('', function () use ($app) {
	$app->get('/', function (Request $request, Response $response, array $args) {
		return $response->withRedirect('/home', 301);

	});
	$app->get('/home', function (Request $request, Response $response, array $args) {
		// $viewParam = $request->getAttribute('viewParam');
		return $this->view->render($response, '/Main.php');
	});
	$app->get('/astar', function (Request $request, Response $response, array $args) {
		// $viewParam = $request->getAttribute('viewParam');
		return $this->view->render($response, '/astar.js');
	});
});

$app->group('/move', function () use ($app) {
	$app->get('', function (Request $request, Response $response, array $args) {
	    $move = new Move($this->db);
	    $ack = $move->getMove($request->getParsedBody());
	    $response = $response->withHeader('Content-type', 'application/json' );
		$response = $response->withJson($ack);
	    return $response ;
	});
	$app->post('', function (Request $request, Response $response, array $args) {
	    $move = new Move($this->db);
	    $ack = $move->insert($request->getParsedBody());
	    $response = $response->withHeader('Content-type', 'application/json' );
		$response = $response->withJson($ack);
	    return $response ;
	});
});
	
$app->run();


